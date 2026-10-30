#!/usr/bin/env bash
# Source after ROS setup, before creating nodes. Changes this shell's children
# only; no OS/shared-memory cleanup, middleware replacement, or .bashrc edits.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'Source this helper: source scripts/dds_env.bash [PROFILE_XML_OR_DIR]\n' >&2
  exit 2
fi

smart_vision_dds_env() {
  local mode=${SMART_VISION_DDS_TRANSPORT:-udp}
  local publication_mode=${SMART_VISION_DDS_PUBLICATION_MODE:-}
  local helper_dir profile=${1:-} prefix profile_name=dds_udp.xml
  case "$mode" in
    udp|default) ;;
    *) printf 'SMART_VISION_DDS_TRANSPORT must be udp or default.\n' >&2; return 2 ;;
  esac
  case "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" in
    rmw_fastrtps_cpp|rmw_fastrtps_dynamic_cpp) ;;
    *) return 0 ;; # A Fast DDS profile must not configure another middleware.
  esac
  case "$publication_mode" in
    ''|ASYNCHRONOUS|SYNCHRONOUS|AUTO) ;;
    *) printf 'SMART_VISION_DDS_PUBLICATION_MODE must be ASYNCHRONOUS, SYNCHRONOUS, AUTO, or empty.\n' >&2
       return 2 ;;
  esac
  # Publication mode is independent of transport/XML selection. Preserve any
  # explicit middleware setting, including an explicitly empty caller value.
  if [[ -n "$publication_mode" && ! ${RMW_FASTRTPS_PUBLICATION_MODE+x} ]]; then
    export RMW_FASTRTPS_PUBLICATION_MODE="$publication_mode"
  fi
  if [[ "$mode" == default ]]; then
    # Undo only values installed by this helper. Keep later user replacements.
    if [[ -n "${_SMART_VISION_DDS_OWN_PROFILE:-}" ]]; then
      if [[ "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" == "$_SMART_VISION_DDS_OWN_PROFILE" ]]; then
        unset FASTRTPS_DEFAULT_PROFILES_FILE
      fi
      if [[ "${FASTDDS_DEFAULT_PROFILES_FILE:-}" == "$_SMART_VISION_DDS_OWN_PROFILE" ]]; then
        unset FASTDDS_DEFAULT_PROFILES_FILE
      fi
      unset _SMART_VISION_DDS_OWN_PROFILE
    fi
    return 0
  fi
  # Explicit site/user DDS profiles take precedence, including inherited ones.
  if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
    if [[ -n "${_SMART_VISION_DDS_OWN_PROFILE:-}"
          && "${FASTRTPS_DEFAULT_PROFILES_FILE:-$_SMART_VISION_DDS_OWN_PROFILE}" == "$_SMART_VISION_DDS_OWN_PROFILE"
          && "${FASTDDS_DEFAULT_PROFILES_FILE:-$_SMART_VISION_DDS_OWN_PROFILE}" == "$_SMART_VISION_DDS_OWN_PROFILE" ]]; then
      # A web wrapper can change localhost scope after sourcing sim_env.
      # Re-select only our own profile; never replace a caller's later edits.
      unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE _SMART_VISION_DDS_OWN_PROFILE
    else
      return 0
    fi
  fi
  if [[ "${ROS_LOCALHOST_ONLY:-0}" == 1 ]]; then profile_name=dds_udp_localhost.xml; fi
  helper_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || return
  if [[ -d "$profile" ]]; then profile=$profile/$profile_name; fi
  if [[ -z "$profile" ]]; then
    if [[ -f "$helper_dir/$profile_name" ]]; then
      profile=$helper_dir/$profile_name
    elif [[ -f "$helper_dir/../ros2_ws/src/smart_vision_amr/config/$profile_name" ]]; then
      profile=$helper_dir/../ros2_ws/src/smart_vision_amr/config/$profile_name
    elif command -v ros2 >/dev/null && prefix=$(ros2 pkg prefix smart_vision_amr 2>/dev/null); then
      profile=$prefix/share/smart_vision_amr/config/$profile_name
    fi
  fi
  if [[ -z "$profile" || ! -r "$profile" ]]; then
    printf 'UDP DDS profile is missing. Rebuild smart_vision_amr or copy dds_env.bash and both dds_udp*.xml profiles together.\n' >&2
    return 2
  fi
  profile=$(realpath -- "$profile") || return
  export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
  export FASTDDS_DEFAULT_PROFILES_FILE="$profile"
  export _SMART_VISION_DDS_OWN_PROFILE="$profile"
}

smart_vision_dds_env "$@"
