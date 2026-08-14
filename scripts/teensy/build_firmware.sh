#!/usr/bin/env bash
set -euo pipefail

# Compile the exact checked-in Teensy 4.1 source without uploading it. The
# installed Teensy 1.60 post-build GUI/security helpers require newer glibc than
# the Heron computer, so only those auxiliary hooks are disabled. The ARM
# compiler, linker, objcopy, libraries, and board flags remain canonical.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/../.." && pwd)"
catkin_workspace="$(cd "${package_dir}/../../.." && pwd)"
cli="${ARDUINO_CLI:-/snap/arduino-cli/current/usr/bin/arduino-cli}"
data_dir="${ARDUINO_DATA_DIR:-${HOME}/.arduino15}"
library_source="${ARDUINO_LIBRARY_SOURCE:-${HOME}/Arduino/libraries}"
output_dir="${1:-${package_dir}/build/teensy41}"
fqbn="teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us"

[[ -x "${cli}" ]] || { echo "arduino-cli is not executable: ${cli}" >&2; exit 2; }
[[ -f "${library_source}/RTClib/library.properties" ]] || {
  echo "RTClib is missing from ${library_source}" >&2; exit 2;
}
[[ -f "${library_source}/Adafruit_BusIO/library.properties" ]] || {
  echo "Adafruit BusIO is missing from ${library_source}" >&2; exit 2;
}
grep -qx 'version=2.1.4' "${library_source}/RTClib/library.properties" || {
  echo "RTClib must be exactly 2.1.4" >&2; exit 2;
}
grep -qx 'version=1.17.4' "${library_source}/Adafruit_BusIO/library.properties" || {
  echo "Adafruit BusIO must be exactly 1.17.4" >&2; exit 2;
}

tmp_dir="$(mktemp -d)"
cleanup() { rm -rf -- "${tmp_dir}"; }
trap cleanup EXIT
stage="${tmp_dir}/main"
user_dir="${tmp_dir}/arduino-user"
mkdir -p "${stage}" "${user_dir}/libraries" "${output_dir}"
cp "${package_dir}"/main/* "${stage}/"
cp "${package_dir}/config/teensy/firmware_config.h" "${stage}/"
cp -a "${library_source}/RTClib" "${user_dir}/libraries/"
cp -a "${library_source}/Adafruit_BusIO" "${user_dir}/libraries/"
cp -a "${library_source}/ros_lib" "${user_dir}/libraries/"

rosserial_head="$(git -C "${catkin_workspace}/src/rosserial" rev-parse HEAD)"
[[ "${rosserial_head}" == "c169ae2173dcfda7cee567d64beae45198459400" ]] || {
  echo "rosserial must be exactly c169ae2173dcfda7cee567d64beae45198459400" >&2
  exit 2
}
[[ -z "$(git -C "${catkin_workspace}/src/rosserial" status --porcelain)" ]] || {
  echo "rosserial worktree must be clean" >&2; exit 2;
}
ros_lib_sha256="$(
  cd "${user_dir}/libraries/ros_lib"
  while IFS= read -r -d '' relative_path; do
    printf '%s  %s\n' "$(sha256sum "${relative_path}" | awk '{print $1}')" "${relative_path#./}"
  done < <(find . -type f -print0 | sort -z) | sha256sum | awk '{print $1}'
)"
[[ "${ros_lib_sha256}" == "bd37a4063ac213b2a8734a575f59c4c3b54e0818dc45ca2de5349dd63f7d50ff" ]] || {
  echo "ros_lib content hash mismatch: ${ros_lib_sha256}" >&2; exit 2;
}

source_files=(
  "${package_dir}/config/teensy/firmware_config.h"
  "${package_dir}/main/firmware_pin_contract.h"
  "${package_dir}/main/main.ino"
  "${package_dir}/main/sensor_sync.h"
  "${package_dir}/main/sensor_sync_runtime.h"
  "${package_dir}/main/telescope_control.h"
  "${package_dir}/main/telescope_runtime.h"
)
source_sha256="$(
  for source_file in "${source_files[@]}"; do
    relative_path="${source_file#${package_dir}/}"
    printf '%s  %s\n' "$(sha256sum "${source_file}" | awk '{print $1}')" "${relative_path}"
  done | sha256sum | awk '{print $1}'
)"
build_id="source-sha256:${source_sha256}"
printf '%s\n' \
  '#pragma once' \
  'namespace ig_handle_firmware_config {' \
  "static const char kFirmwareBuildId[] = \"${build_id}\";" \
  '}' > "${stage}/firmware_build_identity.h"

cli_version="$(${cli} version | sed -n 's/^arduino-cli  Version: \([^ ]*\).*/\1/p')"
[[ "${cli_version}" == "1.3.0" ]] || {
  echo "arduino-cli must be exactly 1.3.0, found ${cli_version:-unknown}" >&2
  exit 2
}
core_line="$(ARDUINO_DIRECTORIES_DATA="${data_dir}" ARDUINO_DIRECTORIES_USER="${user_dir}" "${cli}" core list | awk '$1=="teensy:avr" {print $2}')"
[[ "${core_line}" == "1.60.0" ]] || {
  echo "Teensy core must be exactly 1.60.0, found ${core_line:-missing}" >&2
  exit 2
}

rm -f -- "${output_dir}"/main.ino.eep "${output_dir}"/main.ino.elf \
  "${output_dir}"/main.ino.hex "${output_dir}"/firmware_manifest.txt
ARDUINO_DIRECTORIES_DATA="${data_dir}" \
ARDUINO_DIRECTORIES_USER="${user_dir}" \
"${cli}" compile \
  --fqbn "${fqbn}" \
  --output-dir "${output_dir}" \
  --build-property recipe.hooks.sketch.prebuild.1.pattern=/bin/true \
  --build-property recipe.hooks.objcopy.postobjcopy.1.pattern=/bin/true \
  --build-property recipe.hooks.postbuild.1.pattern=/bin/true \
  --build-property recipe.hooks.postbuild.2.pattern=/bin/true \
  --build-property recipe.hooks.postbuild.3.pattern=/bin/true \
  --build-property recipe.hooks.postbuild.4.pattern=/bin/true \
  --build-property recipe.hooks.savehex.postsavehex.1.pattern=/bin/true \
  "${stage}"

for artifact in main.ino.elf main.ino.hex; do
  [[ -s "${output_dir}/${artifact}" ]] || {
    echo "missing firmware artifact: ${artifact}" >&2; exit 3;
  }
done
{
  printf 'schema_version=1\n'
  printf 'firmware_build_id=%s\n' "${build_id}"
  printf 'source_sha256=%s\n' "${source_sha256}"
  printf 'fqbn=%s\n' "${fqbn}"
  printf 'arduino_cli_version=%s\n' "${cli_version}"
  printf 'teensy_core_version=%s\n' "${core_line}"
  printf 'rtclib_version=2.1.4\n'
  printf 'adafruit_busio_version=1.17.4\n'
  printf 'rosserial_repository_head=%s\n' "${rosserial_head}"
  printf 'generated_ros_lib_sha256=%s\n' "${ros_lib_sha256}"
  printf 'postbuild_helpers=disabled_glibc_2_31_incompatible_noncompile_hooks\n'
  sha256sum "${output_dir}/main.ino.elf" "${output_dir}/main.ino.hex"
} > "${output_dir}/firmware_manifest.txt"

echo "Firmware compiled without upload: ${output_dir}"
cat "${output_dir}/firmware_manifest.txt"
