#!/usr/bin/env bash

set -Eeuo pipefail

# ============================================================
# ONNX Runtime C/C++ CPU-only installer
#
# Installation layout:
#   /opt/onnxruntime/1.23.2
#   /opt/onnxruntime/current -> /opt/onnxruntime/1.23.2
#
# Expected files:
#   /opt/onnxruntime/current/include/onnxruntime_cxx_api.h
#   /opt/onnxruntime/current/lib/libonnxruntime.so
# ============================================================

ORT_VERSION="${ORT_VERSION:-1.23.2}"
INSTALL_ROOT="${INSTALL_ROOT:-/opt/onnxruntime}"
INSTALL_DIR="${INSTALL_ROOT}/${ORT_VERSION}"
CURRENT_LINK="${INSTALL_ROOT}/current"

log() {
    echo "[ORT INSTALL] $*"
}

error() {
    echo "[ORT INSTALL][ERROR] $*" >&2
    exit 1
}

cleanup() {
    if [[ -n "${WORK_DIR:-}" && -d "${WORK_DIR}" ]]; then
        rm -rf "${WORK_DIR}"
    fi
}

trap cleanup EXIT

# ------------------------------------------------------------
# 1. OS 확인
# ------------------------------------------------------------

if [[ ! -f /etc/os-release ]]; then
    error "/etc/os-release 파일을 찾을 수 없습니다."
fi

# shellcheck disable=SC1091
source /etc/os-release

if [[ "${ID:-}" != "ubuntu" ]]; then
    log "경고: 이 스크립트는 Ubuntu 22.04를 기준으로 작성되었습니다."
fi

if [[ "${VERSION_ID:-}" != "22.04" ]]; then
    log "경고: 현재 Ubuntu 버전은 ${VERSION_ID:-unknown}입니다."
fi

# ------------------------------------------------------------
# 2. CPU 아키텍처 확인
# ------------------------------------------------------------

case "$(uname -m)" in
    x86_64 | amd64)
        ORT_ARCH="x64"
        ;;
    aarch64 | arm64)
        ORT_ARCH="aarch64"
        ;;
    *)
        error "지원하지 않는 CPU 아키텍처입니다: $(uname -m)"
        ;;
esac

ARCHIVE_NAME="onnxruntime-linux-${ORT_ARCH}-${ORT_VERSION}.tgz"
DOWNLOAD_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ARCHIVE_NAME}"

log "ONNX Runtime version : ${ORT_VERSION}"
log "Architecture         : ${ORT_ARCH}"
log "Install directory    : ${INSTALL_DIR}"
log "Download URL         : ${DOWNLOAD_URL}"

# ------------------------------------------------------------
# 3. 필수 패키지 설치
# ------------------------------------------------------------

log "필수 패키지를 설치합니다."

sudo apt-get update
sudo apt-get install -y \
    ca-certificates \
    curl \
    tar

# ------------------------------------------------------------
# 4. 이미 설치되어 있는지 확인
# ------------------------------------------------------------

if [[ -f "${INSTALL_DIR}/include/onnxruntime_cxx_api.h" &&
      -f "${INSTALL_DIR}/lib/libonnxruntime.so" ]]; then
    log "ONNX Runtime ${ORT_VERSION}이 이미 설치되어 있습니다."
else
    WORK_DIR="$(mktemp -d)"
    ARCHIVE_PATH="${WORK_DIR}/${ARCHIVE_NAME}"

    log "ONNX Runtime을 다운로드합니다."
    curl \
        --fail \
        --location \
        --show-error \
        --retry 3 \
        --output "${ARCHIVE_PATH}" \
        "${DOWNLOAD_URL}"

    log "다운로드한 압축 파일을 확인합니다."
    tar -tzf "${ARCHIVE_PATH}" >/dev/null

    log "압축을 해제합니다."
    tar -xzf "${ARCHIVE_PATH}" -C "${WORK_DIR}"

    EXTRACTED_DIR="${WORK_DIR}/onnxruntime-linux-${ORT_ARCH}-${ORT_VERSION}"

    if [[ ! -d "${EXTRACTED_DIR}" ]]; then
        error "압축 해제 디렉터리를 찾을 수 없습니다: ${EXTRACTED_DIR}"
    fi

    if [[ ! -f "${EXTRACTED_DIR}/include/onnxruntime_cxx_api.h" ]]; then
        error "ONNX Runtime C++ 헤더를 찾을 수 없습니다."
    fi

    if [[ ! -f "${EXTRACTED_DIR}/lib/libonnxruntime.so" ]]; then
        error "ONNX Runtime 공유 라이브러리를 찾을 수 없습니다."
    fi

    log "${INSTALL_DIR}에 설치합니다."

    sudo mkdir -p "${INSTALL_ROOT}"
    sudo rm -rf "${INSTALL_DIR}"
    sudo cp -a "${EXTRACTED_DIR}" "${INSTALL_DIR}"
fi

# ------------------------------------------------------------
# 5. current 심볼릭 링크 갱신
# ------------------------------------------------------------

log "current 심볼릭 링크를 갱신합니다."

sudo ln -sfn "${INSTALL_DIR}" "${CURRENT_LINK}"

# ------------------------------------------------------------
# 6. 동적 라이브러리 경로 등록
# ------------------------------------------------------------

LD_CONF="/etc/ld.so.conf.d/onnxruntime.conf"

log "동적 라이브러리 경로를 등록합니다: ${LD_CONF}"

echo "${CURRENT_LINK}/lib" |
    sudo tee "${LD_CONF}" >/dev/null

sudo ldconfig

# ------------------------------------------------------------
# 7. 설치 검증
# ------------------------------------------------------------

HEADER_PATH="${CURRENT_LINK}/include/onnxruntime_cxx_api.h"
LIBRARY_PATH="${CURRENT_LINK}/lib/libonnxruntime.so"

[[ -f "${HEADER_PATH}" ]] ||
    error "헤더 설치 검증에 실패했습니다: ${HEADER_PATH}"

[[ -e "${LIBRARY_PATH}" ]] ||
    error "라이브러리 설치 검증에 실패했습니다: ${LIBRARY_PATH}"

if [[ ! -L "${CURRENT_LINK}" ]]; then
    error "current 심볼릭 링크가 올바르게 생성되지 않았습니다."
fi

RESOLVED_LIBRARY="$(readlink -f "${LIBRARY_PATH}")"

[[ -f "${RESOLVED_LIBRARY}" ]] ||
    error "라이브러리 심볼릭 링크 대상이 존재하지 않습니다: ${RESOLVED_LIBRARY}"

sudo ldconfig

if ldconfig -p | grep -qi "onnxruntime"; then
    log "ldconfig 캐시에서 ONNX Runtime을 확인했습니다."
else
    log "경고: ldconfig 캐시에서 ONNX Runtime이 표시되지 않습니다."
    log "직접 링크 검증을 계속합니다."
fi

if ! ldd "${RESOLVED_LIBRARY}" >/dev/null; then
    error "ONNX Runtime 라이브러리 의존성 검사에 실패했습니다."
fi

log "설치가 완료되었습니다."

echo
echo "Version directory:"
echo "  ${INSTALL_DIR}"
echo
echo "Current link:"
echo "  ${CURRENT_LINK} -> $(readlink -f "${CURRENT_LINK}")"
echo
echo "Header:"
echo "  ${HEADER_PATH}"
echo
echo "Library link:"
echo "  ${LIBRARY_PATH}"
echo
echo "Resolved library:"
echo "  ${RESOLVED_LIBRARY}"
echo
echo "Library cache:"
ldconfig -p | grep -i "onnxruntime" || \
    echo "  ldconfig cache entry not shown; direct library path is available."