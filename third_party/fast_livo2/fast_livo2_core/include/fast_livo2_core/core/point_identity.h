#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <limits>
namespace fastlivo_identity {
struct Source {
  uint64_t packet=0, stamp_ns=0;
  std::string adapted_message_sha256;
  uint32_t input_count=0;
};
struct Ref {
  std::shared_ptr<const Source> source;
  uint32_t input_ordinal=0;
};
inline void validate(const std::vector<Ref>&refs,size_t n) {
  if(refs.size()!=n)throw std::runtime_error("point identity cardinality mismatch");
  for(const auto&r:refs)if(!r.source||!r.source->packet||!r.source->stamp_ns||
      r.source->adapted_message_sha256.size()!=64||r.input_ordinal>=r.source->input_count)
    throw std::runtime_error("point identity source invalid");
}
inline std::vector<Ref> selected(std::shared_ptr<const Source>source,
    const std::vector<uint32_t>&indices,size_t n) {
  std::vector<Ref>out;out.reserve(indices.size());
  for(auto i:indices)out.push_back({source,i});validate(out,n);return out;
}
inline void append_ref(std::vector<Ref>&cur,std::vector<Ref>&next,const Ref&ref,bool current) {
  (current?cur:next).push_back(ref);
}
inline void carry(std::vector<Ref>&cur,std::vector<Ref>&next) {cur=std::move(next);next.clear();}
inline std::string binary(const std::vector<Ref>&refs) {
  // Same explicit LE bytes, with one bounded allocation and indexed stores.
  if(refs.size()>std::numeric_limits<size_t>::max()/12)throw std::length_error("point identity binary size");
  std::string out(refs.size()*12,'\0');char* dest=out.data();
  for(const auto&r:refs){const auto packet=r.source->packet;const auto ordinal=r.input_ordinal;
    for(unsigned j=0;j<8;++j)dest[j]=char((packet>>(8*j))&255);
    for(unsigned j=0;j<4;++j)dest[8+j]=char((ordinal>>(8*j))&255);
    dest+=12;}
  return out;
}
}
