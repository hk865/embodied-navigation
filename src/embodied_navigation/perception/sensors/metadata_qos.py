"""Corrected feedback subscription depth; ordinary typed QoS configuration."""
KEY='r74_corrected_metadata_latest_only'
def subscription_depth(profile):
    option=profile.get(KEY)
    if option is None:return 10
    if not isinstance(option,dict)or type(option.get('enabled'))is not bool:
        raise ValueError('Latest-metadata option must be boolean')
    return 1 if option['enabled']else 10
