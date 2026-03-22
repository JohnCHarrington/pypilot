#!/bin/sh
set -eu

usage() {
    echo "Usage: $0 mcp251x|mcp251xfd [bitrate]"
    echo "  mcp251x   = MCP2515 boards"
    echo "  mcp251xfd = MCP2517FD/MCP2518FD boards"
    echo "  bitrate   = optional, default 250000"
    exit 1
}

DRIVER="${1:-}"
BITRATE="${2:-250000}"

case "$DRIVER" in
    mcp251x|mcp251xfd) ;;
    *) usage ;;
esac

ARCH="$(uname -m)"
KVER="$(uname -r)"
TC_MAJOR="13.x"
TC_ARCH="armv7"
TC_MIRROR="http://tinycorelinux.net/${TC_MAJOR}/${TC_ARCH}"
TCEDIR="/etc/sysconfig/tcedir"
OPTIONAL="${TCEDIR}/optional"
ONBOOT="${TCEDIR}/onboot.lst"
WORK="/tmp/can-install.$$"

if [ "$ARCH" != "armv7l" ]; then
    echo "This script expects armv7l. Found: $ARCH"
    exit 1
fi

case "$KVER" in
    *piCore-v7) ;;
    *)
        echo "Unexpected kernel: $KVER"
        echo "This script is intended for piCore-v7 kernels."
        exit 1
        ;;
esac

cleanup() {
    rm -rf "$WORK"
}
trap cleanup EXIT INT TERM

mkdir -p "$WORK" "$OPTIONAL"
[ -f "$ONBOOT" ] || touch "$ONBOOT"

echo "Loading build/runtime dependencies..."
tce-load -wi compiletc git squashfs-tools iproute2 linux-5.10.y_api_headers wget

echo "Downloading matching kernel modules tarball..."
MOD_TAR="${WORK}/${KVER}_modules.tar.xz"
MOD_URL="${TC_MIRROR}/releases/RPi/src/kernel/${KVER}_modules.tar.xz"
wget -O "$MOD_TAR" "$MOD_URL"

echo "Extracting kernel modules tarball..."
mkdir -p "${WORK}/modules"
tar -xf "$MOD_TAR" -C "${WORK}/modules"

echo "Packaging CAN kernel modules..."
PKG_MOD="${WORK}/pkg-mod"
MOD_BASE_SRC="${WORK}/modules/modules/lib/modules/${KVER}"
MOD_BASE_DST="${PKG_MOD}/usr/local/lib/modules/${KVER}"

mkdir -p "${MOD_BASE_DST}/kernel/net/can"
mkdir -p "${MOD_BASE_DST}/kernel/drivers/net/can/dev"
mkdir -p "${MOD_BASE_DST}/kernel/drivers/net/can/spi"
mkdir -p "${MOD_BASE_DST}/kernel/drivers/net/can/spi/mcp251xfd"
mkdir -p "${PKG_MOD}/usr/local/tce.installed"

cp "${MOD_BASE_SRC}/kernel/net/can/can.ko" \
   "${MOD_BASE_DST}/kernel/net/can/"
cp "${MOD_BASE_SRC}/kernel/net/can/can-raw.ko" \
   "${MOD_BASE_DST}/kernel/net/can/"
cp "${MOD_BASE_SRC}/kernel/drivers/net/can/dev/can-dev.ko" \
   "${MOD_BASE_DST}/kernel/drivers/net/can/dev/"

if [ "$DRIVER" = "mcp251x" ]; then
    cp "${MOD_BASE_SRC}/kernel/drivers/net/can/spi/mcp251x.ko" \
       "${MOD_BASE_DST}/kernel/drivers/net/can/spi/"
    DRIVER_KO="kernel/drivers/net/can/spi/mcp251x.ko"
else
    cp "${MOD_BASE_SRC}/kernel/drivers/net/can/spi/mcp251xfd/mcp251xfd.ko" \
       "${MOD_BASE_DST}/kernel/drivers/net/can/spi/mcp251xfd/"
    DRIVER_KO="kernel/drivers/net/can/spi/mcp251xfd/mcp251xfd.ko"
fi

MOD_EXT="can-modules-${KVER}-${DRIVER}"
MOD_INSTALL="${PKG_MOD}/usr/local/tce.installed/${MOD_EXT}"

cat > "$MOD_INSTALL" <<EOF
#!/bin/sh
KVER="${KVER}"
BASE="/usr/local/lib/modules/\$KVER"

insmod "\$BASE/kernel/net/can/can.ko" 2>/dev/null || true
insmod "\$BASE/kernel/net/can/can-raw.ko" 2>/dev/null || true
insmod "\$BASE/kernel/drivers/net/can/dev/can-dev.ko" 2>/dev/null || true
insmod "\$BASE/${DRIVER_KO}" 2>/dev/null || true
EOF
chmod 775 "$MOD_INSTALL"

MOD_TCZ="${OPTIONAL}/${MOD_EXT}.tcz"
rm -f "$MOD_TCZ"
mksquashfs "$PKG_MOD" "$MOD_TCZ" -noappend >/dev/null

if ! grep -qxF "${MOD_EXT}.tcz" "$ONBOOT"; then
    echo "${MOD_EXT}.tcz" >> "$ONBOOT"
fi

echo "Loading extensions now..."
tce-load -i "$MOD_TCZ"

echo "Trying to bring up can0 at ${BITRATE}..."
if ip link show can0 >/dev/null 2>&1; then
    ip link set can0 down >/dev/null 2>&1 || true
    ip link set can0 up type can bitrate "$BITRATE" || true
else
    echo "can0 does not exist yet."
    echo "That usually means the SPI/CAN overlay is still missing or incorrect."
fi

echo
echo "Install complete."
echo "Persistent extensions:"
echo "  ${MOD_TCZ}"
echo
echo "Added to:"
echo "  ${ONBOOT}"
echo
echo "Post-install checks:"
echo "  tce-status -i | grep -E 'can-utils|can-modules'"
echo "  lsmod | grep -E 'can|mcp251'"
echo "  ip -details link show can0"
echo "  dmesg | grep -i mcp251"
echo "  candump can0"