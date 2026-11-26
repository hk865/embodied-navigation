"""FastAPI local vision foundation model server (SAM 3 / YOLOE-seg + optional CLIP/SigLIP).

All heavy GPU dependencies (PyTorch, Ultralytics, open_clip, FastAPI, Uvicorn) are
imported lazily inside constructors/factory functions so the core ``perception``
package and unit tests remain 100% pure CPU.
"""

from __future__ import annotations

import base64
import binascii
import io
from pathlib import Path
import sys
import threading
import time
from typing import Any

from PIL import Image

from embodied_navigation.perception.vision_tools import (
    MAX_MASK_PIXELS,
    encode_mask_b64,
)

_MAX_IMAGE_B64_CHARS = 16 * 1024 * 1024


def _decode_image_b64(image_b64: str) -> Image.Image:
    if len(image_b64) > _MAX_IMAGE_B64_CHARS:
        raise ValueError("image_payload_too_large")
    try:
        raw = base64.b64decode(image_b64.encode("ascii"), validate=True)
    except (UnicodeEncodeError, binascii.Error, ValueError) as exc:
        raise ValueError("invalid_base64_image") from exc
    with Image.open(io.BytesIO(raw)) as img:
        w, h = img.size
        if w <= 0 or h <= 0 or w * h > MAX_MASK_PIXELS:
            raise ValueError(f"image_dimensions_exceed_limit:{w}x{h}")
        return img.convert("RGB")


def _crop_box(img: Image.Image, box_xyxy: list[float], pad_frac: float = 0.08) -> Image.Image:
    w, h = img.size
    x0, y0, x1, y1 = (
        float(box_xyxy[0]),
        float(box_xyxy[1]),
        float(box_xyxy[2]),
        float(box_xyxy[3]),
    )
    bw, bh = max(2.0, x1 - x0), max(2.0, y1 - y0)
    cx0 = max(0, int(round(x0 - bw * pad_frac)))
    cy0 = max(0, int(round(y0 - bh * pad_frac)))
    cx1 = min(w, max(cx0 + 2, int(round(x1 + bw * pad_frac))))
    cy1 = min(h, max(cy0 + 2, int(round(y1 + bh * pad_frac))))
    return img.crop((cx0, cy0, cx1, cy1))


class Sam3Backend:
    """Open-vocabulary concept segmentation powered by local SAM 3."""

    name = "sam3"

    def __init__(self, source_dir: str, checkpoint: str, device: str) -> None:
        import torch

        src = Path(source_dir).resolve()
        if not src.is_dir():
            raise FileNotFoundError(f"sam3 source dir not found: {src}")
        if str(src) not in sys.path:
            sys.path.insert(0, str(src))
        from sam3.model.sam3_image_processor import Sam3Processor
        from sam3.model_builder import build_sam3_image_model

        self._torch = torch
        self._device = str(device)
        # Upstream build_sam3_image_model only moves on device == "cuda"; explicitly move to exact target device
        self._model = build_sam3_image_model(
            checkpoint_path=str(Path(checkpoint).resolve()),
            load_from_HF=False,
            device=self._device,
            eval_mode=True,
        )
        self._model = self._model.to(self._device).eval()
        self._processor = Sam3Processor(
            self._model, device=self._device, confidence_threshold=0.25
        )

    def segment_text(
        self,
        img: Image.Image,
        prompts: list[str],
        *,
        score_threshold: float,
        max_detections: int,
        include_masks: bool,
    ) -> list[dict[str, Any]]:
        torch = self._torch
        w, h = img.size
        dets: list[dict[str, Any]] = []
        self._processor.confidence_threshold = float(score_threshold)
        use_amp = self._device.startswith("cuda")
        with torch.inference_mode(), torch.amp.autocast(
            "cuda", dtype=torch.bfloat16, enabled=use_amp
        ):
            state = self._processor.set_image(img)
            for prompt in prompts:
                out = self._processor.set_text_prompt(prompt=prompt, state=state)
                scores = out.get("scores")
                boxes = out.get("boxes")
                masks = out.get("masks")
                if scores is None or boxes is None or scores.numel() == 0:
                    continue
                s_np = scores.detach().float().cpu().numpy().reshape(-1)
                b_np = boxes.detach().float().cpu().numpy().reshape(-1, 4)
                m_np = None
                if include_masks and masks is not None and masks.numel() > 0:
                    m_np = masks.detach().float().cpu().numpy() > 0.5
                    if m_np.ndim == 4:
                        m_np = m_np[:, 0]
                for idx in range(len(s_np)):
                    sc = float(s_np[idx])
                    if sc < float(score_threshold):
                        continue
                    bx = [
                        max(0.0, min(float(w), float(b_np[idx][0]))),
                        max(0.0, min(float(h), float(b_np[idx][1]))),
                        max(0.0, min(float(w), float(b_np[idx][2]))),
                        max(0.0, min(float(h), float(b_np[idx][3]))),
                    ]
                    entry: dict[str, Any] = {
                        "label": prompt,
                        "score": round(sc, 4),
                        "box_xyxy": bx,
                    }
                    if m_np is not None and idx < len(m_np) and m_np[idx].shape == (h, w):
                        entry["mask"] = encode_mask_b64(m_np[idx])
                    dets.append(entry)
        dets.sort(key=lambda d: -float(d["score"]))
        return dets[: max(1, int(max_detections))]


class YoloeBackend:
    """Fast open-vocabulary detection & segmentation powered by Ultralytics YOLOE."""

    name = "yoloe"

    def __init__(self, model_path: str, device: str, imgsz: int = 512) -> None:
        from ultralytics import YOLOE

        self._model = YOLOE(str(model_path))
        self._model.to(device)
        self._device = str(device)
        self._imgsz = int(imgsz)
        self._pe_cache: dict[tuple[str, ...], Any] = {}
        self._active_key: tuple[str, ...] | None = None

    def segment_text(
        self,
        img: Image.Image,
        prompts: list[str],
        *,
        score_threshold: float,
        max_detections: int,
        include_masks: bool,
    ) -> list[dict[str, Any]]:
        key = tuple(prompts)
        if key != self._active_key:
            pe = self._pe_cache.get(key)
            if pe is None:
                pe = self._model.get_text_pe(list(key))
                if len(self._pe_cache) >= 32:
                    self._pe_cache.clear()
                self._pe_cache[key] = pe
            self._model.set_classes(list(key), pe)
            self._active_key = key

        # retina_masks=True aligns segmentation masks cleanly to the unpadded original (H, W) image
        res_list = self._model.predict(
            img,
            imgsz=self._imgsz,
            conf=float(score_threshold),
            device=self._device,
            retina_masks=True,
            verbose=False,
        )
        if not res_list:
            return []
        r = res_list[0]
        if r.boxes is None or len(r.boxes) == 0:
            return []
        w, h = img.size
        cls_np = r.boxes.cls.detach().cpu().numpy().astype(int).reshape(-1)
        conf_np = r.boxes.conf.detach().cpu().numpy().astype(float).reshape(-1)
        xyxy_np = r.boxes.xyxy.detach().cpu().numpy().astype(float).reshape(-1, 4)
        masks_np = (
            (r.masks.data.detach().cpu().numpy() > 0.5)
            if (include_masks and r.masks is not None and r.masks.data is not None)
            else None
        )
        dets: list[dict[str, Any]] = []
        for i in range(len(conf_np)):
            cid = int(cls_np[i])
            if cid < 0 or cid >= len(key):
                continue
            bx = [
                max(0.0, min(float(w), float(xyxy_np[i][0]))),
                max(0.0, min(float(h), float(xyxy_np[i][1]))),
                max(0.0, min(float(w), float(xyxy_np[i][2]))),
                max(0.0, min(float(h), float(xyxy_np[i][3]))),
            ]
            entry: dict[str, Any] = {
                "label": key[cid],
                "score": round(float(conf_np[i]), 4),
                "box_xyxy": bx,
            }
            if masks_np is not None and i < len(masks_np) and masks_np[i].shape == (h, w):
                entry["mask"] = encode_mask_b64(masks_np[i])
            dets.append(entry)
        dets.sort(key=lambda d: -float(d["score"]))
        return dets[: max(1, int(max_detections))]


class GroundingDinoBackend:
    """Open-vocabulary object detection via local HuggingFace/ModelScope GroundingDINO."""

    name = "grounding_dino"

    def __init__(self, model_dir: str, device: str) -> None:
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        path = str(Path(model_dir).resolve())
        self._torch = torch
        self._device = str(device)
        self._processor = AutoProcessor.from_pretrained(path, local_files_only=True)
        self._model = AutoModelForZeroShotObjectDetection.from_pretrained(
            path, local_files_only=True
        ).to(self._device).eval()

    def segment_text(
        self,
        img: Image.Image,
        prompts: list[str],
        *,
        score_threshold: float,
        max_detections: int,
        include_masks: bool,
    ) -> list[dict[str, Any]]:
        _ = include_masks  # GroundingDINO outputs 2D boxes only (range_source=vision_box_depth_fallback)
        torch = self._torch
        w, h = img.size
        text_query = ". ".join(p.rstrip(".") for p in prompts) + "."
        inputs = self._processor(images=img, text=text_query, return_tensors="pt").to(self._device)
        use_amp = self._device.startswith("cuda")
        with torch.inference_mode(), torch.amp.autocast(
            "cuda", dtype=torch.bfloat16, enabled=use_amp
        ):
            outputs = self._model(**inputs)
        results = self._processor.post_process_grounded_object_detection(
            outputs,
            inputs["input_ids"],
            threshold=float(score_threshold),
            text_threshold=float(score_threshold),
            target_sizes=[(h, w)],
        )
        if not results:
            return []
        res = results[0]
        scores = res["scores"].detach().float().cpu().tolist()
        boxes = res["boxes"].detach().float().cpu().tolist()
        labels = res.get("text_labels") or res.get("labels") or []
        dets: list[dict[str, Any]] = []
        for sc, bx, lbl in zip(scores, boxes, labels):
            clean_lbl = str(lbl).strip().lower()
            if not clean_lbl or float(sc) < float(score_threshold):
                continue
            dets.append(
                {
                    "label": clean_lbl,
                    "score": round(float(sc), 4),
                    "box_xyxy": [
                        max(0.0, min(float(w), float(bx[0]))),
                        max(0.0, min(float(h), float(bx[1]))),
                        max(0.0, min(float(w), float(bx[2]))),
                        max(0.0, min(float(h), float(bx[3]))),
                    ],
                }
            )
        dets.sort(key=lambda d: -float(d["score"]))
        return dets[: max(1, int(max_detections))]


class HfSiglipEmbedder:
    """Local HuggingFace/ModelScope SigLIP continuous image & text embedder."""

    def __init__(self, model_dir: str, device: str) -> None:
        import torch
        from transformers import AutoModel, AutoProcessor

        path = Path(model_dir).resolve()
        self.model_name = path.name
        self._torch = torch
        self._device = str(device)
        self._processor = AutoProcessor.from_pretrained(str(path), local_files_only=True)
        self._model = AutoModel.from_pretrained(str(path), local_files_only=True).to(self._device).eval()

    @staticmethod
    def _extract_tensor(out: Any) -> Any:
        if hasattr(out, "pooler_output") and out.pooler_output is not None:
            return out.pooler_output
        return out

    def embed_images(self, images: list[Image.Image]) -> list[list[float]]:
        if not images:
            return []
        torch = self._torch
        inputs = self._processor(images=images, return_tensors="pt").to(self._device)
        use_amp = self._device.startswith("cuda")
        with torch.inference_mode(), torch.amp.autocast(
            "cuda", dtype=torch.bfloat16, enabled=use_amp
        ):
            feats = self._extract_tensor(self._model.get_image_features(**inputs))
            feats = feats / feats.norm(dim=-1, keepdim=True).clamp_min(1e-12)
        return [[round(float(v), 5) for v in row] for row in feats.float().cpu().tolist()]

    def embed_texts(self, texts: list[str]) -> list[list[float]]:
        if not texts:
            return []
        torch = self._torch
        inputs = self._processor(
            text=texts, padding="max_length", truncation=True, return_tensors="pt"
        ).to(self._device)
        use_amp = self._device.startswith("cuda")
        with torch.inference_mode(), torch.amp.autocast(
            "cuda", dtype=torch.bfloat16, enabled=use_amp
        ):
            feats = self._extract_tensor(self._model.get_text_features(**inputs))
            feats = feats / feats.norm(dim=-1, keepdim=True).clamp_min(1e-12)
        return [[round(float(v), 5) for v in row] for row in feats.float().cpu().tolist()]


class ClipEmbedder:
    """Continuous vision-language image & text embedder via ``open_clip``."""

    def __init__(
        self, model_name: str, pretrained: str, cache_dir: str | None, device: str
    ) -> None:
        import open_clip
        import torch

        self.model_name = str(model_name)
        self._torch = torch
        self._device = str(device)
        self._model, _, self._preprocess = open_clip.create_model_and_transforms(
            model_name, pretrained=pretrained, cache_dir=cache_dir or None
        )
        self._tokenizer = open_clip.get_tokenizer(model_name)
        self._model = self._model.to(self._device).eval()

    def embed_images(self, images: list[Image.Image]) -> list[list[float]]:
        if not images:
            return []
        torch = self._torch
        batch = torch.stack([self._preprocess(im) for im in images], dim=0).to(self._device)
        use_amp = self._device.startswith("cuda")
        with torch.inference_mode(), torch.amp.autocast(
            "cuda", dtype=torch.bfloat16, enabled=use_amp
        ):
            feats = self._model.encode_image(batch)
            feats = feats / feats.norm(dim=-1, keepdim=True).clamp_min(1e-12)
        return [[round(float(v), 5) for v in row] for row in feats.float().cpu().tolist()]

    def embed_texts(self, texts: list[str]) -> list[list[float]]:
        if not texts:
            return []
        torch = self._torch
        toks = self._tokenizer(texts).to(self._device)
        use_amp = self._device.startswith("cuda")
        with torch.inference_mode(), torch.amp.autocast(
            "cuda", dtype=torch.bfloat16, enabled=use_amp
        ):
            feats = self._model.encode_text(toks)
            feats = feats / feats.norm(dim=-1, keepdim=True).clamp_min(1e-12)
        return [[round(float(v), 5) for v in row] for row in feats.float().cpu().tolist()]


def create_vision_app(
    *,
    seg_backend: Any = None,
    embedder: Any = None,
    device: str = "cuda",
) -> Any:
    """Create a FastAPI application with strict Pydantic validation and serialized GPU lock."""
    from typing import Union

    from fastapi import Body, FastAPI, HTTPException
    from pydantic import (
        BaseModel,
        Field,
        StrictBool,
        StrictFloat,
        StrictInt,
        StrictStr,
        ValidationError,
    )

    class SegmentTextRequest(BaseModel):
        image_b64: StrictStr = Field(min_length=1, max_length=_MAX_IMAGE_B64_CHARS)
        prompts: list[StrictStr] = Field(min_length=1, max_length=16)
        score_threshold: Union[StrictFloat, StrictInt] = Field(default=0.25, ge=0.0, le=1.0)
        max_detections: StrictInt = Field(default=12, ge=1, le=32)
        include_masks: StrictBool = True
        include_crop_embeddings: StrictBool = False

    class EmbedImageRequest(BaseModel):
        image_b64: StrictStr = Field(min_length=1, max_length=_MAX_IMAGE_B64_CHARS)

    class EmbedTextRequest(BaseModel):
        text: StrictStr = Field(min_length=1, max_length=512)

    # Resolve local type annotations cleanly under ``from __future__ import annotations``
    ns = {
        "StrictStr": StrictStr,
        "StrictFloat": StrictFloat,
        "StrictInt": StrictInt,
        "StrictBool": StrictBool,
        "Union": Union,
    }
    SegmentTextRequest.model_rebuild(_types_namespace=ns)
    EmbedImageRequest.model_rebuild(_types_namespace=ns)
    EmbedTextRequest.model_rebuild(_types_namespace=ns)

    def _validate(model_cls: type[BaseModel], payload: Any) -> Any:
        try:
            return model_cls.model_validate(payload)
        except ValidationError as exc:
            raise HTTPException(status_code=422, detail=exc.errors()) from None

    lock = threading.Lock()
    app = FastAPI(title="Embodied Navigation Vision Tool Service")

    @app.get("/health")
    def health() -> dict[str, Any]:
        return {
            "ok": True,
            "backend": (seg_backend.name if seg_backend is not None else "none"),
            "embed_model": (embedder.model_name if embedder is not None else ""),
            "device": device,
        }

    @app.post("/segment_text")
    def segment_text(payload: Any = Body(...)) -> dict[str, Any]:
        if seg_backend is None:
            raise HTTPException(status_code=503, detail="segmentation_backend_disabled")
        req = _validate(SegmentTextRequest, payload)
        prompts = list(dict.fromkeys(p.strip().lower() for p in req.prompts if p.strip()))
        if not prompts:
            raise HTTPException(status_code=400, detail="empty_prompts")
        t0 = time.perf_counter()
        with lock:
            try:
                img = _decode_image_b64(req.image_b64)
            except (ValueError, OSError):
                raise HTTPException(status_code=400, detail="invalid_image") from None
            dets = seg_backend.segment_text(
                img,
                prompts,
                score_threshold=float(req.score_threshold),
                max_detections=int(req.max_detections),
                include_masks=bool(req.include_masks),
            )
            if req.include_crop_embeddings and embedder is not None and dets:
                crops = [_crop_box(img, d["box_xyxy"]) for d in dets[: int(req.max_detections)]]
                embs = embedder.embed_images(crops)
                for det, emb in zip(dets, embs):
                    det["embedding"] = emb
        return {
            "backend": seg_backend.name,
            "latency_ms": round((time.perf_counter() - t0) * 1000.0, 1),
            "detections": dets,
        }

    @app.post("/embed_image")
    def embed_image(payload: Any = Body(...)) -> dict[str, Any]:
        if embedder is None:
            raise HTTPException(status_code=503, detail="embedder_disabled")
        req = _validate(EmbedImageRequest, payload)
        with lock:
            try:
                img = _decode_image_b64(req.image_b64)
            except (ValueError, OSError):
                raise HTTPException(status_code=400, detail="invalid_image") from None
            emb = embedder.embed_images([img])[0]
        return {"embed_model": embedder.model_name, "embedding": emb}

    @app.post("/embed_text")
    def embed_text(payload: Any = Body(...)) -> dict[str, Any]:
        if embedder is None:
            raise HTTPException(status_code=503, detail="embedder_disabled")
        req = _validate(EmbedTextRequest, payload)
        clean = req.text.strip()
        if not clean:
            raise HTTPException(status_code=400, detail="empty_text")
        with lock:
            emb = embedder.embed_texts([clean])[0]
        return {"embed_model": embedder.model_name, "embedding": emb}

    return app


def run_vision_service(
    *,
    backend: str = "sam3",
    sam3_source: str = "",
    sam3_checkpoint: str = "",
    yoloe_model: str = "",
    gdino_model: str = "",
    embed_model: str = "",
    embed_pretrained: str = "openai",
    embed_cache_dir: str = "",
    device: str = "cuda",
    host: str = "127.0.0.1",
    port: int = 8114,
) -> int:
    """Instantiate configured local vision backends and serve over HTTP."""
    import uvicorn

    seg_backend: Any = None
    if backend == "sam3":
        if not sam3_source or not sam3_checkpoint:
            raise ValueError("--backend sam3 requires --sam3-source and --sam3-checkpoint")
        seg_backend = Sam3Backend(sam3_source, sam3_checkpoint, device)
    elif backend == "yoloe":
        if not yoloe_model:
            raise ValueError("--backend yoloe requires --yoloe-model")
        seg_backend = YoloeBackend(yoloe_model, device)
    elif backend == "grounding_dino":
        if not gdino_model:
            raise ValueError("--backend grounding_dino requires --gdino-model")
        seg_backend = GroundingDinoBackend(gdino_model, device)

    embedder: Any = None
    if embed_model:
        if Path(embed_model).is_dir():
            embedder = HfSiglipEmbedder(embed_model, device)
        else:
            embedder = ClipEmbedder(embed_model, embed_pretrained, embed_cache_dir or None, device)
    app = create_vision_app(seg_backend=seg_backend, embedder=embedder, device=device)
    uvicorn.run(app, host=host, port=int(port), log_level="warning")
    return 0
