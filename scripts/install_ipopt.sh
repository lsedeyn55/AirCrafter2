#!/bin/bash
# Build Ipopt (with MUMPS) into the project's virtual environment and install cyipopt against it.
# Optional: only robust.optimizer: ipopt (RDO) needs it. No root needed; uses the system BLAS/LAPACK and gfortran.
#
#   scripts/install_ipopt.sh              # venv = .venv, build tree = third_party/ipopt_build (git-ignored)
#
# Libraries go to <venv>/lib, headers to <venv>/include/coin-or, pkg-config files to <venv>/lib/pkgconfig;
# cyipopt finds them through an rpath, so deleting the venv removes everything. The rpath is absolute: after
# moving the project folder, rerun this script.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
VENV=${VENV:-$ROOT/.venv}
BUILD=${BUILD:-$ROOT/third_party/ipopt_build}
IPOPT_TAG=releases/3.14.20
MUMPS_TAG=releases/3.0.14
CYIPOPT=cyipopt==1.7.0
JOBS=${JOBS:-8}

mkdir -p "$BUILD"
cd "$BUILD"
[ -d ThirdParty-Mumps ] || git clone -q --depth 1 -b $MUMPS_TAG https://github.com/coin-or-tools/ThirdParty-Mumps
[ -d Ipopt ] || git clone -q --depth 1 -b $IPOPT_TAG https://github.com/coin-or/Ipopt

cd ThirdParty-Mumps
[ -d MUMPS ] || ./get.Mumps
[ -f Makefile ] && make distclean -s >/dev/null 2>&1 || true
./configure --prefix="$VENV" -q
make -j"$JOBS" -s && make install -s
cd ..

rm -rf Ipopt/build && mkdir -p Ipopt/build && cd Ipopt/build
PKG_CONFIG_PATH="$VENV/lib/pkgconfig" ../configure --prefix="$VENV" --disable-java -q
make -j"$JOBS" -s && make install -s

PKG_CONFIG_PATH="$VENV/lib/pkgconfig" LDFLAGS="-Wl,-rpath,$VENV/lib" \
    uv pip install --python "$VENV/bin/python" --reinstall-package cyipopt --no-cache --no-binary cyipopt "$CYIPOPT"
"$VENV/bin/python" -c "import cyipopt; print('cyipopt', cyipopt.__version__, 'OK')"
