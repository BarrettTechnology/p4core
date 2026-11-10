# make test -- this library's unit tests (tests/), on every CPU when
# pytest-xdist is installed (requirements-test.txt), else one by one.
#
# The tests import the package by name from this directory's parent, so
# check out this repo as a directory named p4core. An app's venv has every
# dependency:
#
#   make test PYTHON=../pucktuner/.venv/bin/python
#
# PYTHON (default python3), TEST_JOBS (default auto: one per CPU).

PYTHON ?= python3
TEST_JOBS ?= auto
# The canbar test opens a wx window: on a private Xvfb when xvfb-run is
# installed, as in the apps' Makefiles.
MESA_EGL := $(wildcard /usr/share/glvnd/egl_vendor.d/50_mesa.json)
XVFB := $(if $(shell command -v xvfb-run 2>/dev/null),env $(if $(MESA_EGL),__EGL_VENDOR_LIBRARY_FILENAMES=$(MESA_EGL)) \
	xvfb-run -a -s "-screen 0 1280x1024x24 -noreset" env -u WAYLAND_DISPLAY GDK_BACKEND=x11)
# Threaded BLAS only spins on the tests' small matrices.
export OPENBLAS_NUM_THREADS = 1
export OMP_NUM_THREADS = 1

.PHONY: test help

test:
	@if $(PYTHON) -c 'import xdist' 2>/dev/null; then \
	    $(XVFB) $(PYTHON) -m pytest -n $(TEST_JOBS) -q -p no:cacheprovider tests; \
	else \
	    echo "pytest-xdist is not installed (pip install -r requirements-test.txt): running the tests one by one." >&2; \
	    $(XVFB) $(PYTHON) -m unittest discover -s tests; \
	fi

help:
	@sed -n '1,/^$$/p' Makefile
