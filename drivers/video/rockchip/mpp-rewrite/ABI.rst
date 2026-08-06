Rockchip MPP Rewrite ABI
========================

Scope
-----

The rewrite registers ``/dev/mpp_service`` when
``ROCKCHIP_MPP_REWRITE`` is enabled and the BSP MPP service is disabled.  It
implements the current RK3588 libmpp ABI for:

=============  =========  ==============  ==================
Client         ID          Hardware        Register-0 ID
=============  =========  ==============  ==================
RKVENC2        ``16``      VEPU580         ``0x50603312``
RKVDEC2        ``9``       VDPU381         ``0x53813f05``
AV1 decoder    ``4``       RK3588 AV1      ``0x80019000``
=============  =========  ==============  ==================

This document describes the userspace contract.  Internal locking, recovery,
KUnit coverage, and implementation history are intentionally kept out of it.

The read-only ``/proc/mpp_service/supports-cmd`` and compatibility alias
``support_cmd`` enumerate the supported command values.  The read-only
``supports-device`` entry uses the BSP text format to list each currently
usable device type and its validated hardware ID.  Its contents track the same
live support state reported by ``MPP_CMD_QUERY_HW_SUPPORT`` and
``MPP_CMD_QUERY_HW_ID``.

Transport
---------

``MPP_IOC_CFG_V1`` is the supported ioctl.  Its ioctl encoding is
``_IOW('v', 1, unsigned int)``, but the argument points to this fixed-width
24-byte record for both native and compat callers::

  u32 cmd;
  u32 flags;
  u32 size;
  u32 offset;
  u64 data_ptr;

Without ``MPP_FLAGS_MULTI_MSG`` the argument contains one record.  With it,
records are contiguous and ``MPP_FLAGS_LAST_MSG`` terminates the ioctl.  The
hard limit is 64 records per ioctl and 16 staged send/poll requests per job.
Register jobs in a batch are submitted before later polls are processed,
matching libmpp's trigger-then-wait ordering.

Write-like payloads (register writes, address offsets, and RCB descriptors) are
copied during ioctl collection.  A register-read pointer is retained until
the consuming poll has copied readback and must remain valid through that
poll.  Each request payload and each materialized register image is bounded to
128 KiB; a job may contain up to 16 requests.  Custom translation tables and
address-offset arrays contain at most 80 entries.

Unknown commands and command-boundary sentinels are rejected before payload
processing; the collected V1 ioctl reports ``-EFAULT`` for that parser failure.
Unknown flag bits and flags used with the wrong command class return
``-EINVAL``.  ``MPP_IOC_CFG_V2`` is not supported.

Flags
-----

- ``MPP_FLAGS_MULTI_MSG`` and ``MPP_FLAGS_LAST_MSG`` are transport flags
  accepted on every command.
- ``MPP_FLAGS_REG_FD_NO_TRANS``, ``MPP_FLAGS_SCL_FD_NO_TRANS``, and
  ``MPP_FLAGS_REG_OFFSET_ALONE``/``MPP_FLAGS_REG_NO_OFFSET`` are accepted only
  on send-register and poll commands.  ``MPP_FLAGS_SCL_FD_NO_TRANS`` is
  retained for BSP transport
  compatibility and has no separate fixed-backend behavior.
- ``MPP_FLAGS_POLL_NON_BLOCK`` is accepted only on poll commands.
- ``MPP_FLAGS_SECURE_MODE`` is recognized but returns ``-EOPNOTSUPP`` before
  processing.

Supported commands
------------------

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Command
     - Behavior
   * - ``MPP_CMD_QUERY_HW_SUPPORT``
     - Returns ``BIT(4)``, ``BIT(9)``, and/or ``BIT(16)`` for currently usable
       AV1DEC, RKVDEC2, and RKVENC2 cores.  Required coordinators must also be
       online.
   * - ``MPP_CMD_QUERY_HW_ID``
     - Returns the validated register-0 ID.  Before client initialization, the
       input ``u32`` selects the client; afterwards the bound type does.
   * - ``MPP_CMD_QUERY_CMD_SUPPORT``
     - Replaces a known group-base ``u32`` with its ``*_BUTT`` boundary;
       unknown bases return zero.
   * - ``MPP_CMD_INIT_CLIENT_TYPE``
     - Binds the session client.  Repeating the same type succeeds; rebinding
       returns ``-EBUSY``.
   * - ``MPP_CMD_INIT_DRIVER_DATA``
     - Initialized-session ``u32`` no-op.
   * - ``MPP_CMD_INIT_TRANS_TABLE``
     - Stores 0..80 additional ``u16`` address-register indices.  Built-in
       RK3588 tables always remain active.
   * - ``MPP_CMD_SET_REG_WRITE``
     - Copies a register span into the staged job.
   * - ``MPP_CMD_SET_REG_READ``
     - Records a bounded readback span.
   * - ``MPP_CMD_SET_REG_ADDR_OFFSET``
     - Adds up to 80 ``{ u32 index, u32 offset }`` tuples.
   * - ``MPP_CMD_SET_RCB_INFO``
     - Stores ``{ u32 index, u32 size }`` scratch descriptors; application is
       filtered by the trusted hardware layout.
   * - ``MPP_CMD_SET_SESSION_FD``
     - Switches following messages to another open MPP session.
   * - ``MPP_CMD_POLL_HW_FINISH``
     - Waits for and consumes the oldest pending job.
   * - ``MPP_CMD_POLL_HW_IRQ``
     - Streams RKVENC2 slice results; otherwise uses finish behavior.
   * - ``MPP_CMD_RESET_SESSION``
     - Cancels staged/queued/active work and releases cached imports.
   * - ``MPP_CMD_TRANS_FD_TO_IOVA``
     - Maps an exact array of 1..80 DMA-BUF fds and overwrites it with IOVAs.
   * - ``MPP_CMD_RELEASE_FD``
     - Immediately unlinks cached mappings for an exact array of 1..80 fds;
       actual unmap is deferred while a job retains a reference.
   * - ``MPP_CMD_SEND_CODEC_INFO``
     - Stores valid ``{ u32 type, u32 flag, u64 data }`` elements.  Unsupported
       elements and trailing bytes are tolerated for BSP compatibility.
   * - ``MPP_CMD_SET_ERR_REF_HACK``
     - Initialized-session copy-in/discard; zero length is allowed and the
       payload is bounded to one page.

``SET_SESSION_FD`` uses the fixed 16-byte payload
``{ u64 flag, u32 fd, s32 ret }``.  ``MPP_BAT_MSG_DONE`` skips the remainder of
that message group through its ``LAST_MSG`` marker.  An invalid integer fd
writes ``-EBADF`` to that entry's ``ret`` and skips its group without failing
the whole ioctl; a live non-MPP fd returns ``-EINVAL``.

Sessions and job lifetime
-------------------------

Register writes, read descriptors (including the retained userspace pointer),
offsets, RCB descriptors, client type, translation table, and codec information
are recorded in job state.  Successful ``INIT_CLIENT_TYPE``,
``INIT_TRANS_TABLE``, and ``SEND_CODEC_INFO`` changes split later messages into
a new staged job, so they cannot alter earlier work.  ``SET_RCB_INFO`` belongs
to the current staged job and is snapshotted on submission.

``INIT_DRIVER_DATA``, ``RESET_SESSION``, ``TRANS_FD_TO_IOVA``,
``SEND_CODEC_INFO``, and ``SET_ERR_REF_HACK`` require an initialized session.
``INIT_TRANS_TABLE`` may precede client initialization.  Staged register work
is rejected at admission if the session remains uninitialized.

Accepted jobs remain on the session pending list until a consuming FINISH or
terminal IRQ poll removes their result, or reset/close aborts them.  Contended
submissions queue inside the driver; they do not wait for a core in the submit
ioctl.

``RESET_SESSION`` advances the session generation, discards older staged work,
cancels queued/active work, and empties the import cache.  Work racing the reset
is rejected with ``-ECANCELED``.  Closing the file performs the same cleanup.

There is no aggregate per-session byte/count quota beyond the per-request and
per-ioctl caps above.  Cached imports and pending unconsumed jobs can grow until
release, consuming poll, reset, or close.

``SET_SESSION_FD`` accepts only another MPP service fd.  It finalizes the
current staged-job boundary without discarding that job, then directs following
register/poll messages to the selected session.  A later group may switch back
to an earlier session.

DMA-BUF and IOVA rules
----------------------

All codec-visible addresses are 32-bit.  A mapped DMA-BUF must be nonempty and
cover its complete size as one byte-contiguous DMA span inside that aperture.
The physical backing may contain multiple SG entries; the device-visible span
may not contain gaps.

Kernel-translated jobs
  Address-table register values normally encode the fd in the low 10 bits and
  an embedded byte offset in the high 22 bits.  ``REG_NO_OFFSET`` makes the
  entire ``u32`` a plain fd; offsets then come only from
  ``SET_REG_ADDR_OFFSET``.  The driver maps fds on the selected core's DMA
  device, writes IOVAs into the private image, and retains every mapping
  through completion.

Explicit-IOVA jobs
  ``TRANS_FD_TO_IOVA`` pins one matching DMA device as the session's affinity.
  A job using ``MPP_FLAGS_REG_FD_NO_TRANS`` is accepted only on that device,
  and every nonzero address in the built-in plus custom translation table must
  fall inside a retained session mapping.  An offline affinity returns
  ``-ENODEV``; the driver does not silently select another IOMMU domain.
  Rebinding the same platform device restores selection without invalidating
  its cached mappings.

The import cache is keyed by fd, DMA device, and DMA-BUF object.  Reusing an
integer fd for a different object cannot return the old mapping.
``RELEASE_FD`` is serialized with translation and cannot invalidate an IOVA
already retained by a job.  Releasing the last cached explicit mapping clears
the session affinity; reset and close also clear it.  Translation/release
payloads are nonempty exact ``u32`` arrays with no trailing bytes.

Translation and release arrays are processed in order, not transactionally.
A later bad fd leaves earlier successful translations cached or releases
already performed.  Translation copyout failure also leaves completed mappings
cached, although explicit-device affinity is published only after successful
copyout.  Actual unmap remains deferred while a job holds the import.

The fixed per-client address tables are always unioned with the custom table;
known address registers cannot be made literal by omission.  AV1 retains all
103 built-in translation entries even though the custom table is limited to 80.
Optional zero address values remain valid, but later offsets may not turn one
into an unowned literal IOVA.

Offsets apply in order with checked cumulative arithmetic.  Ordinary address
registers reject ``offset >= dma_buf_size``.  The only exception is a
driver-translated RKVENC2 VEPU580 bitstream-top word 172, which may equal
``base + size`` because it is an end-exclusive limit.  Explicit literal IOVA
lookup is strict even for word 172; all other registers, RKVDEC2, and AV1 are
strict as well.  A literal non-address register may receive an additive offset
only when the ``u32`` value does not wrap.

The driver neither waits on nor publishes DMA-BUF reservation fences and has
no sync-file command.  External producers must finish before submission, and
userspace must not reuse codec buffers until the corresponding consuming poll
has established completion.

Register images
---------------

The declared userspace regions are:

===========  ====================  =========================
Backend      Class                 Span
===========  ====================  =========================
RKVENC2      dense                 ``[0, 0x6000)``
RKVDEC2      dense                 ``[0, 0x5a0)``
RKVDEC2      performance selector  ``[0x20000, 0x20100)``
AV1          VCD                   ``[0, 0x800)``
AV1          cache                 ``[0x10000, 0x10298)``
AV1          AFBC                  ``[0x20000, 0x20350)``
===========  ====================  =========================

A request must be aligned and fit entirely inside one class.  Holes,
cross-class spans, truncated readback, and arithmetic overflow are rejected
before MMIO.  The AV1 output AFBC header and payload must fit the same retained
DMA-BUF provenance as output register 505.

Multiple write spans materialize in message order.  Start registers are written
only after the remaining validated image.  Successful read requests return the
retained hardware result, including the BSP interrupt-status substitutions and
decoder decoded-length adjustment.  RKVDEC2 performance-selector reads use the
logical side window above and return the three selected value words; that range
is not ordinary writable MMIO.  AV1 AFBC request bytes may be retained for
readback but are not written directly to the driver-owned AFBC block.

RCB scratch
-----------

``SET_RCB_INFO`` applies to RKVENC2 and RKVDEC2.  It stores up to four encoder
or sixteen decoder ``(register index, size)`` pairs and snapshots them per job.
A pair is used only when the index exists in the device-tree hardware RCB
layout and the nonzero size does not exceed that entry's trusted maximum.
Unknown, zero-sized, and oversized entries are ignored without consuming
scratch space.  The first accepted pair for an index reserves that entry's
complete trusted maximum, regardless of the smaller requested size; later
pairs for the same trusted index are ignored without consuming space.

Accepted entries are patched with driver-owned coherent IOVAs after normal
translation and offset processing.  Final validation accepts only the exact
kernel-owned value.  Decoder RCB also obeys ``rockchip,rcb-min-width`` using
the retained codec width.  Complete coherent allocations must fit the 32-bit
aperture; DMA address zero is valid.

Polling
-------

``POLL_HW_FINISH`` waits for and consumes the oldest pending job.  Successful
jobs copy requested readback; failed jobs return their status without copying
it.  ``MPP_FLAGS_POLL_NON_BLOCK`` returns ``-EAGAIN`` while that job is
incomplete without consuming it.  Polling an empty session returns ``-EIO``.
A staged job accepts at most one poll command; a second ``POLL_HW_FINISH`` or
``POLL_HW_IRQ`` in that job is rejected with ``-EINVAL``.
A finish result is consumed before readback copyout, so ``-EFAULT`` loses that
result and retry observes the next job or ``-EIO``.  A blocking poll may return
restart semantics only when interrupted before it changes poll-visible state.
After a batch submits work or any poll consumes a completion or slice record,
an interrupt is returned as ``-EINTR`` so ioctl restart cannot duplicate a
submission or consume the following result.

For VEPU580 slice mode, ``POLL_HW_IRQ`` uses::

  s32 poll_type;
  s32 poll_ret;
  s32 count_max;
  s32 count_ret;
  u32 slice_info[];

When a nonempty result buffer is supplied, ``count_max`` must be positive and
the payload size must cover that many entries.  ``count_ret`` is written after
each successfully copied entry; if no entry is copied, its userspace value is
unchanged.  A poll copies at most ``count_max`` queued slice lengths; bit 31
marks the final slice, after which normal completion/readback is consumed.

A zero-size or null-data request is also accepted in slice mode as an explicit
discard operation: slice records are removed without copying them.  A blocking
call continues toward final completion.  A nonblocking call may discard all
currently queued non-final records and then return ``-EAGAIN``; a final record
is likewise removed before the completion poll.  Nonblocking buffered poll
returns ``-EAGAIN`` only when no record is available.  FIFO overflow is reported once as
``-EOVERFLOW``; retained entries may be drained on retry, and overflow is
deferred when the same call already returned records.  For a non-slice job the
command behaves like ``POLL_HW_FINISH`` and does not parse the flexible buffer.
Each slice record is removed before userspace copyout; a copy fault therefore
loses that record.

Scheduling and hardware modes
-----------------------------

For kernel-translated jobs, the exact compatible online core is an
implementation choice.  Explicit-IOVA jobs stay on their affinity device.  One
job executes at a time on each core.

RKVDEC2 CCU behavior comes from ``rockchip,ccu-mode``:

SOFT (``1``)
  Default, including missing or invalid values.  The coordinator schedules
  work and the selected core executes its register image directly.

HARD (``2``)
  Opt-in linked-table execution.  Every online decoder behind the coordinator
  must share one DMA/IOMMU domain (or all use no IOMMU), and each core must
  expose the required link window.  Otherwise decoder support fails closed.

The shipped Rock 5B device tree selects SOFT mode.  HARD mode is implemented
but remains a separate hardware-validation target.

Completion, timeout, and removal
--------------------------------

Normal completion is IRQ-driven.  The software timeout is 500 ms.  After
successful DMA containment, a timeout completes with ``-ETIMEDOUT`` and an
IOMMU fault with ``-EIO``.  If the driver cannot prove that DMA stopped, it
retains the active job and its DMA resources in fail-stop isolation rather than
reporting false completion; encoder DCHS ownership is retained with that job.
Reset failure quarantines the affected core or coordinator for the bound
instance; terminal IOMMU-group isolation may require a reboot before reprobe
can succeed.  Removal returns ``-ENODEV`` to work that can no longer execute.

The driver does not export or import sync-file fences.  The RK3588 MPP UAPI has
no fence command; userspace orders work through submission and poll.

Validation status
-----------------

RKVENC2 and SOFT-CCU RKVDEC2 have historical Rock 5B runtime evidence, but the
corrected working tree still requires a fresh booted qualification run.  HARD
CCU and AV1 output-AFBC register handling are implemented, source-audited, and
have compiled KUnit coverage, but remain separate hardware-validation targets.
AV1 AFBC qualification also requires proof that downstream AFBC DMA has retired
before completion is exposed.

Discovery and debugging
-----------------------

``/proc/mpp_service/supports-cmd`` and its legacy alias ``support_cmd`` expose
the read-only BSP-style labelled hexadecimal command table, including group
boundary rows, used by libmpp.

``/sys/kernel/debug/rk_mpp_rewrite`` provides:

``state``
  Bound hardware, queue/active state, live import count, and aggregate/per-core
  job, IRQ, timeout, fault, reset, rejection, and timing counters.

``events``
  The latest 64 request, queue, dispatch, start, IRQ, completion, timeout,
  fault, abort, and rejection records.  Writing ``1`` clears retained rows.

``trace_mask``
  Optional live tracing: bit 0 job lifecycle, bit 1 IRQ, bit 2
  rejection/error/recovery.  The default is zero.

Debugfs is diagnostic and is not a stable userspace ABI.

Recognized but unsupported
--------------------------

- ``MPP_IOC_CFG_V2``.
- ``MPP_FLAGS_SECURE_MODE``; there is no protected attachment, secure IOMMU
  domain, or secure-monitor submission path.
- Dormant libmpp batch-server wait arrays containing multiple
  ``SET_SESSION_FD`` + nonblocking ``POLL_HW_FINISH|LAST_MSG`` groups; these
  return ``-EOPNOTSUPP`` without writing a batch status slot.
- Sync-file fence import/export.
- Client classes and register profiles other than RK3588 RKVENC2, RKVDEC2, and
  AV1 listed above.
