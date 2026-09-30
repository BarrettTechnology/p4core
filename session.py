"""session.py — one CAN bus, the pucks on it, and the SYNC that drives them.

Session is the model every app shares.  It owns:

  * the canopen Network (a P4Network: FD per frame, kernel-timed cyclic
    sends, a TX queue that is retried instead of failing an SDO),
  * the selected node and the pucks the last scan found,
  * the SYNC producer, outright (see set_sync_rate: nothing else may call
    network.sync.start()),
  * node resets, saves and version reads, which the apps used to do with
    raw NMT frames from inside their GUI handlers.

It is wx-free and thread-agnostic; the GUI runs it off the UI thread, the
command line runs it inline.  Moved here from p4gui's model.py (P4Model),
which is now a thin subclass.
"""

import time

import can
import canopen

from . import can_backend, paths

class KernelPeriodicTask:
    """Adapts socketcan's kernel cyclic task to the interface canopen expects.

    canopen's PDO map calls task.update(data) to change the payload of a
    running cyclic transmission - that is how an interactive mode moves its
    setpoint without restarting the task.  python-can's native task spells
    that modify_data() and wants a whole Message.  Without this adapter the
    drive handlers raise "'CyclicSendTask' object has no attribute 'update'"
    the moment a mode is entered.
    """

    def __init__(self, task, msg):
        self._task = task
        self._msg = msg

    def update(self, data: bytes) -> None:
        self._msg.data = bytearray(data)
        self._msg.dlc = len(self._msg.data)
        self._task.modify_data(self._msg)

    def stop(self) -> None:
        self._task.stop()


class FdPeriodicMessageTask:
    """Like PeriodicMessageTask but honours the network's can_fd flag.

    Every cyclic producer in the app runs on one of these threads — the SYNC
    and both interactive RPDOs — so the loop below must survive a transmit
    that fails.  See _run() for what happens when it does not.
    """

    #: A send failure is logged at most this often (s), with a running count.
    #: A saturated adapter fails many frames a second; one line per frame
    #: would bury the first, which is the informative one.
    ERROR_REPORT_INTERVAL = 1.0

    def __init__(self, can_id: int, data: bytes, period: float, network, remote: bool = False):
        self._can_id = can_id
        self._data = data
        self._period = period
        self._network = network
        self._remote = remote
        self._stop = False
        self._errors = 0
        self._last_report = 0.0
        import threading
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        next_time = time.monotonic()
        while not self._stop:
            try:
                self._network.send_message(self._can_id, self._data, self._remote)
            except Exception as exc:
                # A transient transmit failure must NOT end the task.  The
                # SocketCAN TX queue is ten frames deep and a USB adapter
                # drains it well below the nominal bitrate (see scan(), which
                # hits the same wall), so a single send can bounce with "No
                # buffer space available"; Network.check() can also re-raise
                # here on behalf of the notifier thread.  Letting either
                # escape kills this thread for the life of the process, and
                # silently: if it is the SYNC producer, every puck stops
                # transmitting its TPDOs at the same instant and the whole bus
                # looks dead, which is indistinguishable from the devices
                # having dropped off it.  Skip the frame; a cyclic producer is
                # allowed to miss one.
                self._report_error(exc)
            next_time += self._period
            now = time.monotonic()
            delay = next_time - now
            if delay > 0:
                time.sleep(delay)
            elif -delay >= self._period:
                # A whole period or more late — the GIL, a blocking SDO on the
                # GUI thread, or the adapter held this thread up.  Re-base on
                # now instead of firing every frame that was missed back to
                # back: a SYNC or cyclic RPDO wants the current time base, not
                # a replay, and that catch-up burst is itself what overruns
                # the TX queue and starts the failure above.
                next_time = now

    #: Printed once per task, with the first failure.  A cyclic producer that
    #: cannot transmit is nearly always the interface rather than the app —
    #: a controller gone bus-off keeps the queue permanently full, which looks
    #: identical to congestion from up here.
    HINT = ('[can]   if this does not stop: ip -details -statistics link show '
            '<dev> (bus-off? error counters climbing?), and the default CAN '
            'txqueuelen of 10 frames is worth raising: ip link set <dev> '
            'txqueuelen 1000')

    def _report_error(self, exc):
        """Log a send failure, rate-limited, so a storm still shows its cause."""
        self._errors += 1
        now = time.monotonic()
        if now - self._last_report < self.ERROR_REPORT_INTERVAL:
            return
        first = self._last_report == 0.0
        self._last_report = now
        print('[can] periodic 0x{:03X}: {} send failure(s), last: {}'
              .format(self._can_id, self._errors, exc))
        if first:
            print(self.HINT)

    def stop(self):
        self._stop = True

    def update(self, data: bytes) -> None:
        self._data = bytes(data)


class P4Network(canopen.Network):
    """canopen.Network that can send CAN FD frames when can_fd is True."""

    #: A transmit rejected because the queue is momentarily full is retried
    #: this many times, this far apart (s), before the error is passed on.
    #: The SocketCAN TX queue is ten frames deep by default; three cyclic
    #: producers at the SYNC rate plus a burst of SDO polling can fill it for
    #: a few hundred microseconds with nothing actually wrong.  Whoever pays
    #: for that is whichever SDO happened to land in the window, and it
    #: surfaces as an unexplained drive timeout several layers up.  A few
    #: milliseconds of backoff covers the drain; a queue still full after that
    #: is a real fault and the caller should hear about it.
    SEND_RETRIES = 3
    SEND_RETRY_DELAY = 0.001

    def __init__(self, bus=None):
        super().__init__(bus)
        self.can_fd = False

    def send_message(self, can_id: int, data: bytes, remote: bool = False) -> None:
        if not self.bus:
            raise RuntimeError("Not connected to CAN bus")
        msg = can.Message(
            is_extended_id=can_id > 0x7FF,
            arbitration_id=can_id,
            data=data,
            is_remote_frame=remote,
            is_fd=self.can_fd,
            bitrate_switch=self.can_fd,
        )
        last_exc = None
        for attempt in range(self.SEND_RETRIES):
            try:
                with self.send_lock:
                    self.bus.send(msg)
            except can.CanError as exc:
                last_exc = exc
                if attempt + 1 < self.SEND_RETRIES:
                    time.sleep(self.SEND_RETRY_DELAY)
                continue
            self.check()
            return
        raise last_exc

    def send_periodic(self, can_id: int, data: bytes, period: float, remote: bool = False):
        """Cyclic transmission, kernel-timed wherever the kernel can do it.

        FdPeriodicMessageTask is a Python thread calling send_message() in a
        loop: it shares send_lock with every SDO transfer and the GIL with the
        GUI, and when it falls a whole period behind it re-bases its schedule,
        which emits one long interval.  For SYNC that is fatal to clock
        discipline - measured on the bench, about 14 % of intervals landed
        more than 25 % off nominal, which is enough for app/clockcal.c to
        spoil most of its measurement windows.  It never locks, the axes
        free-run on untrimmed HSI16, and a purring group walks apart at
        thousands of ppm.

        socketcan's native cyclic send is a kernel BCM timer: once armed it is
        unaffected by anything this process does.  Measured through it under
        three CPU-saturating threads, 0 of 2494 intervals were out of
        tolerance at sd 26 us.

        BCM carries FD frames too - the earlier claim here that it could not
        was wrong.  python-can sets CAN_FD_FRAME in the BCM flags whenever the
        message has is_fd (socketcan.CyclicSendTask._tx_setup), and
        send_periodic returns the same kernel CyclicSendTask, with
        modify_data, either way.  Verified on the bench: FD SYNC through BCM
        drove FD TPDOs from four axes at 0.996 delivery, all four locked.

        Gating this on `not self.can_fd` therefore cost nothing in classic
        mode but silently put SYNC back on the thread the moment FD was
        enabled - i.e. it re-introduced exactly the clock-discipline failure
        described above, in the one configuration nobody had tested.

        Only an interface with no native cyclic support falls back to the
        thread now, and there python-can would spawn one of its own anyway.
        """
        if self.bus is not None:
            msg = can.Message(
                is_extended_id=can_id > 0x7FF,
                arbitration_id=can_id,
                data=data,
                is_remote_frame=remote,
                is_fd=self.can_fd,
                bitrate_switch=self.can_fd,
            )
            try:
                task = self.bus.send_periodic(msg, period)
            except Exception:
                task = None     # No native cyclic support
            if task is not None:
                if hasattr(task, 'modify_data'):
                    return KernelPeriodicTask(task, msg)
                # A native task that cannot be updated in place is no use to
                # canopen's PDO maps; stop it and take the thread instead.
                try:
                    task.stop()
                except Exception:
                    pass
        return FdPeriodicMessageTask(can_id, data, period, self, remote)




#: CiA-301 0x1010 "save" signature.
SAVE_SIGNATURE = 0x65766173


class Session:
    """A CAN bus and the P4 pucks on it."""

    # Fallbacks used only before a node is selected.  A gear ratio of 1.0 is
    # wrong for every real puck, so nothing should command a motor on these —
    # select_node() replaces them with the values read from the device.
    DEFAULT_GEAR_RATIO = 1.0
    DEFAULT_PEAK_VELOCITY = 20000.0

    #: After an NMT reset the flashloader runs first and answers SDO itself;
    #: the application is up after this long (s).  Then select_node() is
    #: retried until REBOOT_TIMEOUT_S.
    BOOT_S = 0.5
    REBOOT_TIMEOUT_S = 10.0

    def __init__(self, eds=None):
        self.eds = eds or paths.puck4_eds()
        self.network = None
        self.node = None
        self.sync_rate = 0
        #: The SYNC task this session started, kept because canopen's own
        #: handle is overwritten by any start() and then cannot stop the
        #: survivor.
        self._sync_task = None
        self.port = None
        self.can_fd = False
        self.fd_available = False
        self.gear_ratio = self.DEFAULT_GEAR_RATIO
        self.peak_velocity = self.DEFAULT_PEAK_VELOCITY
        # Node IDs the last scan found, and the subset that has been fully
        # configured (EDS loaded, PDOs read, OPERATIONAL) — see multi_nodes().
        self.found_ids = []
        self._multi_ready = set()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.disconnect()
        return False

    # ------------------------------------------------------------------
    # Connect
    # ------------------------------------------------------------------
    def connect(self, port, fd=None, bitrate=1_000_000):
        """Connect to the CAN bus on *port* (``can0``; on Windows a PCAN or
        CandleLight index, see can_backend.make_network).

        *fd* None tries CAN FD first and falls back to classic CAN; True or
        False forces one.  Sets fd_available.  Frames go out classic until
        set_can_fd(True) — FD capability and FD use are separate.  Raises
        ConnectionError if no connection could be made.
        """
        self.port = port
        attempts = (True, False) if fd is None else (bool(fd),)
        last = None
        for use_fd in attempts:
            try:
                network = can_backend.make_network(
                    port, bitrate=bitrate, fd=use_fd, network_cls=P4Network)
            except Exception as exc:
                last = exc
                continue
            self.fd_available = use_fd
            self.network = network
            self.network.can_fd = self.can_fd
            return
        raise ConnectionError(str(last)) from last

    @property
    def connected(self):
        return self.network is not None

    def _require_network(self):
        if self.network is None:
            raise ConnectionError('not connected to a CAN bus')
        return self.network

    # ------------------------------------------------------------------
    # Scan
    # ------------------------------------------------------------------
    # Scan tuning.  See scan() for why the sweep is paced rather than blasted.
    SCAN_PASSES = 2        # whole-bus sweeps before giving up
    SCAN_TX_GAP = 0.002    # inter-frame delay during the sweep (s)
    SCAN_TX_RETRIES = 5    # per-frame retries when the TX queue is full
    SCAN_TX_BACKOFF = 0.01 # wait before retrying a frame (s)
    SCAN_SETTLE = 0.5      # max wait for replies after a sweep (s)
    SCAN_QUIET = 0.15      # no new replies for this long ends the wait early (s)

    def scan(self):
        """Scan the bus for CANopen nodes.

        Returns a sorted list of node IDs (ints).  Returns [] if no nodes were
        found.  Raises if the network is not connected.

        Deliberately does NOT use canopen's ``scanner.search()``.  That helper
        pushes 127 SDO requests into the socket back-to-back with no pacing,
        which overruns the SocketCAN TX queue (txqueuelen defaults to 10 frames
        on a can device, and USB adapters drain it far slower than the nominal
        bitrate).  When the queue fills, python-can raises CanOperationError
        ("Failed to transmit: No buffer space available" / "Transmit buffer
        full") from the middle of the sweep, so:

          - node IDs after the overrun never get a request, and
          - the exception propagates out of scan(), discarding replies that had
            already arrived — the puck could be found and still be reported
            missing.

        Whether the overrun happens depends on thread scheduling, so the
        symptom is a scan that works only every few clicks.  puckutility hit
        this too and papered over it by matching "buffer" in the exception text
        and reporting "No Pucks Found".

        So: pace the sweep, retry an individual frame that the queue rejected
        instead of abandoning the sweep, and treat replies already collected as
        valid regardless.
        """
        self._require_network().scanner.reset()
        for _ in range(self.SCAN_PASSES):
            self._search_paced()
            nodes = self._collect_scan_replies()
            if nodes:
                self.found_ids = nodes
                return nodes
        self.found_ids = []
        return []

    def _search_paced(self, limit=127):
        """Send an SDO upload request to every node ID, paced to the TX queue.

        A frame the queue rejects is retried rather than aborting the sweep.
        Only raises if *every* node ID failed, which means the bus is down
        rather than merely busy.
        """
        sdo_req = b'\x40\x00\x10\x00\x00\x00\x00\x00'  # upload 0x1000:0
        last_exc = None
        failures = 0
        for node_id in range(1, limit + 1):
            for _ in range(self.SCAN_TX_RETRIES):
                try:
                    self.network.send_message(0x600 + node_id, sdo_req)
                    break
                except Exception as exc:
                    last_exc = exc
                    time.sleep(self.SCAN_TX_BACKOFF)
            else:
                failures += 1
            time.sleep(self.SCAN_TX_GAP)
        if failures == limit and last_exc is not None:
            raise last_exc

    def _collect_scan_replies(self):
        """Wait for scan replies, returning early once the bus goes quiet.

        The scanner list is appended to by the notifier thread, so this polls it
        rather than sleeping a fixed interval: a lone puck answers in well under
        a millisecond, and waiting the full settle time on every scan is pure
        latency.  The quiet period only short-circuits once something has been
        found, so an empty bus still gets the full window.
        """
        nodes = self.network.scanner.nodes
        now = time.monotonic()
        deadline = now + self.SCAN_SETTLE
        last_change = now
        seen = len(nodes)
        while time.monotonic() < deadline:
            time.sleep(0.02)
            if len(nodes) != seen:
                seen = len(nodes)
                last_change = time.monotonic()
            elif seen and time.monotonic() - last_change >= self.SCAN_QUIET:
                break
        return sorted(nodes)

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------
    def add_node(self, node_id, eds=None):
        """The node for *node_id* with the EDS attached, nothing else done:
        enough for SDO, commands nothing, changes no NMT state."""
        network = self._require_network()
        if node_id not in network:
            network.add_node(node_id, eds or self.eds)
        return network[node_id]

    def raw_node(self, node_id):
        """*node_id* with an EMPTY object dictionary, for raw SDO uploads
        that must not depend on the EDS (version reads, a flashloader).
        Replaces any node already added for that ID; add_node() or
        select_node() puts the EDS back."""
        network = self._require_network()
        self._forget(node_id)
        return network.add_node(node_id, canopen.ObjectDictionary())

    def _forget(self, node_id):
        if self.network is not None and node_id in self.network:
            try:
                del self.network[node_id]
            except Exception:
                pass
        self._multi_ready.discard(node_id)

    def select_node(self, node_id):
        """Make *node_id* the selected node, configured for control, and
        return it.

        EDS attached, PDO configuration read back from the device, heartbeat
        off, OPERATIONAL, and the drive's gear ratio and peak velocity read.
        """
        node = self.add_node(node_id)
        self.node = node
        node.tpdo.read()
        node.rpdo.read()
        node.sdo['HeartbeatPeriod'].raw = 0

        # Selecting a node NMT-resets it (see reset_node).  A puck that comes
        # back PRE-OPERATIONAL processes SDO normally but silently discards
        # every RPDO: ControlWord has no effect, the drive sits in Switch On
        # Disabled, and the frames still look perfectly correct on the wire.
        # A correctly commissioned puck self-starts (0x1F80 = 0) and does not
        # need this, but one whose NVM is stale — e.g. predating a firmware NVM
        # layout change — does, and that failure is near-undiagnosable from the
        # bus.  Starting the node explicitly is a master's job anyway, so do not
        # drop this on the grounds that the device should self-start.
        node.nmt.state = 'OPERATIONAL'
        self._multi_ready.add(node_id)

        self._read_drive_scaling()
        return node

    def _read_drive_scaling(self):
        """Read gear ratio (0x6091) and peak velocity (0x6080) from the device.

        Every torque/velocity/position command and every graph axis is scaled by
        these, so they are read here — at node select — rather than by whichever
        view happens to want them first.

        Deliberately not wrapped in try/except: falling back to a gear ratio of
        1.0 silently over-commands torque by the true ratio (~17.8x on a P4-42),
        which is far worse than failing loudly.  Read by index, so this does not
        depend on the EDS naming these objects.
        """
        motor_rev = int.from_bytes(self.node.sdo.upload(0x6091, 1), 'little')
        shaft_rev = int.from_bytes(self.node.sdo.upload(0x6091, 2), 'little')
        if not shaft_rev:
            raise ValueError(
                'Bad gear ratio from device: 0x6091 shaft revolutions is 0')
        self.gear_ratio = motor_rev / shaft_rev
        self.peak_velocity = float(
            int.from_bytes(self.node.sdo.upload(0x6080, 0), 'little'))

    def require_node(self):
        """The selected node, or a clear error."""
        if self.node is None:
            raise RuntimeError('no node selected')
        return self.node

    # ------------------------------------------------------------------
    # Reset, save, versions
    # ------------------------------------------------------------------
    def send_nmt(self, command, node_id):
        """A raw NMT frame (0x81 reset, 0x82 reset comms, 0x01 start...)."""
        self._require_network().send_message(0x0, [command, node_id])

    def reset_node(self, node_id=None, reselect=None, sleep=time.sleep):
        """NMT-reset *node_id* (default: the selected node) and wait for its
        application to answer again.

        The reset boots the flashloader, which answers SDO itself, and the
        application follows after BOOT_S.  When the node was the selected one
        (or *reselect* is True) it is selected again — the node object is
        reused, but the PDO maps are read back, so callers that registered
        TPDO callbacks must register them again.  Returns the node.
        """
        if node_id is None:
            node_id = self.require_node().id
        if reselect is None:
            reselect = self.node is not None and self.node.id == node_id
        self.send_nmt(0x81, node_id)
        sleep(self.BOOT_S)
        deadline = time.monotonic() + self.REBOOT_TIMEOUT_S
        while True:
            try:
                if reselect:
                    return self.select_node(node_id)
                node = self.add_node(node_id)
                node.sdo.upload(0x1000, 0)
                return node
            except Exception as exc:
                if time.monotonic() > deadline:
                    raise ConnectionError(
                        'node {} did not come back after a reset: {}'.format(
                            node_id, exc)) from exc
                sleep(0.5)

    def save_all(self, node=None):
        """Save every object to NVM (0x1010,1 'save'; takes ~0.55 s)."""
        node = node or self.require_node()
        timeout = node.sdo.RESPONSE_TIMEOUT
        node.sdo.RESPONSE_TIMEOUT = max(timeout, 1.0)
        try:
            node.sdo.download(0x1010, 1, SAVE_SIGNATURE.to_bytes(4, 'little'))
        finally:
            node.sdo.RESPONSE_TIMEOUT = timeout

    def save_single(self, index, sub, node=None):
        """Save one object to NVM (Save.Single takes (index << 8) | sub)."""
        node = node or self.require_node()
        node.sdo['Save']['Single'].raw = (index << 8) | sub

    def read_version(self, node_id=None):
        """The application's 0x100A version string, read raw (EDS-proof)."""
        from .flash import read_version
        if node_id is None:
            node_id = self.require_node().id
        node = self.network[node_id] if node_id in self.network \
            else self.add_node(node_id)
        return read_version(node)

    # ------------------------------------------------------------------
    # Multi-Puck — group control of every puck the last scan found
    # ------------------------------------------------------------------
    def configure_extra_node(self, node_id):
        """Configure *node_id* for group control and return its node object.

        The same setup select_node() gives the selected node — EDS, PDO
        configuration read back from the device, heartbeat off, OPERATIONAL —
        minus becoming self.node and minus the scaling read (commands are
        scaled once, by the selected node's gear ratio; a multi-puck group is
        assumed homogeneous).  Idempotent: a node already configured is
        returned as-is.
        """
        if node_id in self.network and node_id in self._multi_ready:
            return self.network[node_id]
        node = self.add_node(node_id)
        node.tpdo.read()
        node.rpdo.read()
        node.sdo['HeartbeatPeriod'].raw = 0
        # See select_node() for why the node is started explicitly.
        node.nmt.state = 'OPERATIONAL'
        self._multi_ready.add(node_id)
        return node

    def prepare_multi(self):
        """Configure every puck the last scan found; return their nodes.

        PDO read-back is a few dozen SDO transfers per node, so a GUI calls
        this off its UI thread.
        """
        return [self.configure_extra_node(node_id)
                for node_id in self.found_ids]

    def found_nodes(self):
        """Node objects for every puck the last scan found.

        Enough for SDO reads - the EDS is attached - and deliberately not
        configure_extra_node()'s full setup: no PDO read-back, no NMT change,
        nothing commanded.
        """
        if self.network is None:
            return []
        nodes = []
        for node_id in self.found_ids:
            try:
                nodes.append(self.add_node(node_id))
            except Exception:
                continue
        return nodes

    def multi_nodes(self):
        """Node objects for every found-and-configured puck, selected included.

        Only pucks prepare_multi()/select_node() has configured are returned —
        a puck found by a scan but never configured has no EDS or PDO map to
        command through.
        """
        nodes = [self.network[node_id] for node_id in self.found_ids
                 if node_id in self.network and node_id in self._multi_ready]
        if self.node is not None and self.node not in nodes:
            nodes.append(self.node)
        return nodes

    # ------------------------------------------------------------------
    # SYNC and FD
    # ------------------------------------------------------------------
    def set_sync_rate(self, rate_hz):
        """Start or stop the CANopen SYNC at *rate_hz* Hz.

        The session owns the SYNC producer outright, and nothing else may call
        network.sync.start().  canopen's SyncProducer.start() overwrites its
        task handle without stopping the previous task, and on socketcan that
        handle is a kernel BCM timer that goes on transmitting regardless - so
        a second start() anywhere leaves two producers on the bus permanently,
        and the stop() that follows silences only the newer one while canopen's
        handle stops pointing at the survivor.

        Two 100 Hz producers at arbitrary relative phase hand the drives
        intervals of a few milliseconds, far outside the 25 % window
        app/clockcal.c accepts, so every measurement window is spoiled, every
        axis loops LEARNING -> ACQUIRING -> LEARNING and none of them ever
        locks.  Measured on the bus: one start gives 99.9 Hz with 0 of 397
        intervals out of tolerance; a second start gives 199.3 Hz with 397 of
        795 out of tolerance; and stop() afterwards leaves 99.9 Hz still
        flowing.  Undisciplined clocks then free-run on untrimmed HSI16, which
        is seconds of purr divergence over a few minutes.

        Safe to call before connect() - returns immediately if network is None.
        """
        self.sync_rate = rate_hz
        if self.network is None:
            return
        # Stop the task this session started, not merely the one canopen last
        # recorded - they are the same only if nobody bypassed us.
        for task in (self._sync_task, getattr(self.network.sync, '_task', None)):
            if task is not None:
                try:
                    task.stop()
                except Exception:
                    pass
        try:
            self.network.sync.stop()
        except Exception:
            pass
        self._sync_task = None
        if rate_hz > 0:
            self.network.sync.start(1.0 / rate_hz)
            self._sync_task = getattr(self.network.sync, '_task', None)

    def set_can_fd(self, enabled):
        """Set CAN FD message mode.  No reconnection needed."""
        self.can_fd = enabled
        if self.network is not None:
            self.network.can_fd = enabled
            # Restart sync so the new FD flag is picked up by the periodic task
            if self.sync_rate > 0:
                self.set_sync_rate(self.sync_rate)

    # ------------------------------------------------------------------
    # Disconnect
    # ------------------------------------------------------------------
    def disconnect(self):
        """Disconnect from the CAN bus.

        Never raises — safe to call at any point.
        """
        try:
            self.set_sync_rate(0)
            if self.network is not None:
                self.network.disconnect()
        except Exception:
            pass
        self.network = None
        self.node = None
        self.found_ids = []
        self._multi_ready.clear()
