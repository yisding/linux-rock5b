.. SPDX-License-Identifier: GPL-2.0

.. _v4l2-usage-metrics:

==========================
V4L2 client usage metrics
==========================

V4L2 drivers can optionally expose per-file-descriptor usage metrics via
``/proc/<pid>/fdinfo/<fd>``. This is analogous to the DRM fdinfo mechanism
documented in :ref:`drm-client-usage-stats`, but uses the ``v4l2-`` key
prefix for V4L2 devices.

The interface is generic to V4L2: any driver type (stateless or stateful
codecs, capture devices, ISPs, converters, ...) may implement it and provide
the metrics relevant to its hardware. This document defines a common set of
mandatory keys that identify the driver and the kind of device, plus per
driver-type sections describing additional keys.

The initial set of type-specific keys documented here targets stateless
(request API based) codec devices, both decoders and encoders. With
stateless codecs, the kernel driver explicitly submits each frame to the
hardware and receives a completion interrupt, providing a clean per-job
boundary that can be attributed to the submitting file descriptor. Other
device types (stateful codecs, capture devices, ISPs, ...) can be added
later with their own set of type-specific keys.

Implementation
==============

The V4L2 core provides the plumbing: drivers implement the ``show_fdinfo``
callback in ``struct v4l2_file_operations``, and the core wires it into the
kernel ``struct file_operations`` so that ``/proc/<pid>/fdinfo/<fd>`` output
includes the driver-provided keys.

File format specification
=========================

- File shall contain one key value pair per one line of text.
- Colon character (``:``) must be used to delimit keys and values.
- All standardised keys shall be prefixed with ``v4l2-``.
- Driver-specific keys shall be prefixed with ``driver_name-``.

All counter values reported through this interface are cumulative since the
file descriptor was created, and are strictly monotonically increasing.

Mandatory keys
==============

The following keys must be exposed by every driver that implements this
interface, regardless of the device type.

- v4l2-driver: <valstr>

  String shall contain the name of the V4L2 driver.

- v4l2-driver-type: <valstr>

  String shall identify the type of V4L2 device exposed through this file
  descriptor. This key tells userspace which additional type-specific keys
  to expect. Standard values currently defined are ``stateless-decoder``
  and ``stateless-encoder``. Additional values will be defined as this
  interface is extended to other driver types.

Stateless codec keys
====================

The keys described in this section apply to file descriptors whose
``v4l2-driver-type`` is ``stateless-decoder`` or ``stateless-encoder``.

A stateless codec may be composed of one or more independent hardware
cores. Per-core metrics are reported using keys suffixed with a
``<core_id>`` identifier, so a single file descriptor can report metrics
for multiple cores.

``<core_id>`` must be a non-negative decimal integer (e.g. ``0``, ``1``,
``2``, ...). Strings or other non-numeric identifiers are not allowed, so
that userspace can reliably parse and enumerate cores.

Utilization keys
----------------

- v4l2-core-usage-time-<core_id>: <uint> ns

  Mandatory.

  Time in nanoseconds that the hardware core identified by ``<core_id>``
  spent busy processing work belonging to this file descriptor, cumulative
  since the file descriptor was created.

  Time is measured by the driver, typically from just before the hardware
  is started to just after the completion interrupt is handled, so it is
  slightly less precise than a hardware cycle counter but is always
  available.

- v4l2-core-usage-cycles-<core_id>: <uint>

  Optional.

  Number of hardware clock cycles that the core identified by ``<core_id>``
  spent busy processing work belonging to this file descriptor, cumulative
  since the file descriptor was created.

  This is more precise than ``v4l2-core-usage-time-<core_id>`` but requires
  the hardware to expose a cycle counter. When available together with
  ``v4l2-maxfreq-<core_id>`` and ``v4l2-curfreq-<core_id>``, userspace can
  derive an accurate utilization percentage. Otherwise, userspace should
  fall back to ``v4l2-core-usage-time-<core_id>``.

Frequency keys
--------------

- v4l2-maxfreq-<core_id>: <uint> Hz

  Maximum operating frequency of the main clock of the core identified by
  ``<core_id>``.

- v4l2-curfreq-<core_id>: <uint> Hz

  Current operating frequency of the main clock of the core identified by
  ``<core_id>``.

A core may have several clocks associated with it, but only the one that
actually clocks the hardware processing engine of the core (its "main
clock") should be reported through these keys.

Example output
==============

::

  v4l2-driver:                  hantro-vpu
  v4l2-driver-type:             stateless-decoder
  v4l2-core-usage-time-0:       123456789 ns
  v4l2-core-usage-cycles-0:     74000000
  v4l2-maxfreq-0:               600000000 Hz
  v4l2-curfreq-0:               600000000 Hz
