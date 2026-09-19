#!/usr/bin/env python3
# 실주행 1바퀴 계측 하네스.
#   - 파트별/전체 자원 사용량 (CPU, RSS, 디스크 IO, GPU 메모리)
#   - 토픽별 처리율·드롭(seq 결번)·지연(stamp 기준)·바이트
#   - rosbag record 자원 사용량을 따로 찍어 사후 역산 가능
# 사용: python3 cr_monitor.py <출력디렉토리> [측정초] [샘플주기]
import os, re, sys, time, struct, subprocess, threading, csv
import rospy, psutil

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cr_run"
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0      # 0 = Ctrl-C 까지
PERIOD = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

# cmdline 정규식 -> 라벨. 앞쪽이 우선.
PROC_PATTERNS = [
    ("bag_record",      r"rosbag.*\brecord\b|/opt/ros/\S+/lib/rosbag/record"),
    ("camera_start",    r"lib/camera_start/camera_start_node"),
    ("yolo26",          r"yolo_detect\.py"),
    ("velodyne_bev",    r"velodyne_bev_detection\.py"),
    ("livox_cluster",   r"livox_euclidean_clustering\.py"),
    ("livox_fusion",    r"lib/livox_camera_fusion/"),
    ("livox_driver",    r"livox_ros_driver_node"),
    ("velodyne_driver", r"nodelet.*velodyne"),
    ("localizer",       r"lib/localizer/|localizer_node"),
    ("sc_control",      r"[Ss][Cc]_[Cc]ontrol"),
    ("ublox",           r"ublox"),
    ("ntrip",           r"ntrip"),
    ("global_path",     r"global_path"),
    ("local_path",      r"local_path"),
    ("cr_monitor",      r"cr_monitor\.py"),
    ("rosout",          r"lib/rosout/rosout"),
    ("rosmaster",       r"bin/rosmaster"),
    ("roslaunch",       r"bin/roslaunch"),
]

# 커널 스레드는 cmdline 이 비어 있어 프로세스 이름으로 잡는다.
# softirq(네트워크/USB 수신 후처리)와 인터럽트 스레드가 여기 들어간다.
KTHREAD_PATTERNS = [
    ("k_softirq",  r"^ksoftirqd/"),
    ("k_irq",      r"^irq/"),
    ("k_usb",      r"^(usb-storage|kworker/\S*usb|xhci)"),
    ("k_worker",   r"^kworker/"),
]

TOPICS = [
    "/camera/image_raw/compressed",
    "/livox/lidar",
    "/velodyne_points",
    "/perception/camera/yolo",
    "/perception/livox/centroids",
    "/perception/velodyne/centroids",
    "/perception/fusion/centroids",
]

NICS = ["enp8s0", "enp0s31f6"]

# 검증·임시 측정용: CR_TOPICS 로 토픽을 덧붙이거나(기본 목록 유지),
# CR_TOPICS_ONLY 로 목록을 통째로 대체한다. 쉼표 구분.
if os.environ.get("CR_TOPICS_ONLY"):
    TOPICS = [t.strip() for t in os.environ["CR_TOPICS_ONLY"].split(",") if t.strip()]
elif os.environ.get("CR_TOPICS"):
    TOPICS = TOPICS + [t.strip() for t in os.environ["CR_TOPICS"].split(",")
                       if t.strip() and t.strip() not in TOPICS]


class TopicStat(object):
    """AnyMsg 로 받아 타입 의존 없이 센다. 메시지 앞 4바이트가 Header.seq 인
    표준 레이아웃이면 드롭(결번)과 지연(stamp)도 함께 낸다."""
    def __init__(self, topic):
        self.topic = topic
        self.lock = threading.Lock()
        self.n = 0; self.bytes = 0
        self.cum_n = 0; self.cum_bytes = 0; self.cum_drop = 0
        self.drop = 0
        self.last_seq = None
        self.seq_ok = True          # seq 가 말이 되는지
        self.resync = 0             # 발행 노드 재시작 등으로 seq 가 되감긴 횟수
        self.cum_resync = 0
        self.lat_sum = 0.0; self.lat_n = 0; self.lat_max = 0.0
        # 대용량·고주파 토픽(카메라 12MB/s 등)에서 놓치지 않으려면
        # 수신 버퍼를 키워야 한다. rospy 기본 buff_size(64KB)로는 부족하다.
        self.sub = rospy.Subscriber(topic, rospy.AnyMsg, self.cb, queue_size=1000,
                                    buff_size=2 ** 24, tcp_nodelay=True)

    def cb(self, msg):
        buf = msg._buff
        now = time.time()
        with self.lock:
            self.n += 1; self.bytes += len(buf)
            if len(buf) >= 12:
                seq, s, ns = struct.unpack_from("<III", buf, 0)
                if self.last_seq is not None:
                    d = seq - self.last_seq
                    if d <= 0 or d > 100000:
                        # 노드 respawn 으로 seq 가 0 부터 다시 시작하는 경우.
                        # 예전처럼 측정을 영구히 끄지 말고 재동기화만 한다.
                        self.resync += 1
                    elif d > 1:
                        self.drop += d - 1
                self.last_seq = seq
                if 1e9 < s < 4e9:                 # 그럴듯한 epoch 초
                    lat = now - (s + ns * 1e-9)
                    if -1.0 < lat < 30.0:
                        self.lat_sum += lat; self.lat_n += 1
                        if lat > self.lat_max: self.lat_max = lat

    def take(self):
        with self.lock:
            n, b, d, rs = self.n, self.bytes, self.drop, self.resync
            self.resync = 0
            lat = (self.lat_sum / self.lat_n) if self.lat_n else float("nan")
            lmax = self.lat_max
            self.n = self.bytes = self.drop = 0
            self.lat_sum = 0.0; self.lat_n = 0; self.lat_max = 0.0
        self.cum_n += n; self.cum_bytes += b; self.cum_drop += d
        self.cum_resync += rs
        # 재동기화가 전체 수신의 10% 를 넘으면 앞 4바이트가 seq 가 아니라고 본다
        if self.cum_n > 200 and self.cum_resync > 0.1 * self.cum_n:
            self.seq_ok = False
        return n, b, d, lat, lmax, rs


def label_of(cmdline):
    for name, pat in PROC_PATTERNS:
        if re.search(pat, cmdline):
            return name
    return None


_proc_cache = {"t": 0.0, "map": {}}
RESCAN_EVERY = 5.0   # 초. 전수 스캔은 비싸므로 가끔만; 그 사이엔 캐시된 PID 만 읽는다.


def scan_procs_cached():
    """전수 스캔은 RESCAN_EVERY 마다. 그 사이에는 이미 알고 있는 PID 만 확인해
    루프 비용을 낮춘다(구독 스레드가 GIL 을 못 얻어 메시지를 놓치는 것을 방지)."""
    now = time.time()
    if now - _proc_cache["t"] >= RESCAN_EVERY or not _proc_cache["map"]:
        _proc_cache["map"] = scan_procs()
        _proc_cache["t"] = now
    else:
        live = {}
        for lab, procs in _proc_cache["map"].items():
            alive = [p for p in procs if p.is_running()]
            if alive:
                live[lab] = alive
        _proc_cache["map"] = live
    return _proc_cache["map"]


def scan_procs():
    """label -> [psutil.Process]. 매 샘플마다 새로 훑어 재기동(respawn)도 따라간다."""
    out = {}
    for p in psutil.process_iter(["pid", "cmdline", "name"]):
        try:
            cl = " ".join(p.info["cmdline"] or [])
            if cl:
                lab = label_of(cl)
            else:
                nm = p.info["name"] or ""
                lab = None
                for name, pat in KTHREAD_PATTERNS:
                    if re.search(pat, nm):
                        lab = name
                        break
            if lab:
                out.setdefault(lab, []).append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return out


_gpu_cache = {"t": 0.0, "overall": (float("nan"), float("nan")), "per_pid": {}}
GPU_EVERY = 5.0      # 초. nvidia-smi 는 호출당 수십~수백 ms 걸려 루프를 막는다.


def gpu_cached():
    now = time.time()
    if now - _gpu_cache["t"] >= GPU_EVERY:
        _gpu_cache["overall"] = gpu_overall()
        _gpu_cache["per_pid"] = gpu_per_pid()
        _gpu_cache["t"] = now
    return _gpu_cache["overall"], _gpu_cache["per_pid"]


def gpu_overall():
    try:
        o = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
             "--format=csv,noheader,nounits"], timeout=4).decode().strip().splitlines()[0]
        u, m = [x.strip() for x in o.split(",")]
        return float(u), float(m)
    except Exception:
        return float("nan"), float("nan")


def gpu_per_pid():
    try:
        o = subprocess.check_output(
            ["nvidia-smi", "--query-compute-apps=pid,used_memory",
             "--format=csv,noheader,nounits"], timeout=4).decode().strip()
        d = {}
        for line in o.splitlines():
            if not line.strip():
                continue
            pid, mem = [x.strip() for x in line.split(",")]
            d[int(pid)] = float(mem)
        return d
    except Exception:
        return {}


def cpu_stat():
    """/proc/stat 의 전체 CPU 시간(jiffies)을 항목별로."""
    with open("/proc/stat") as f:
        parts = f.readline().split()
    vals = [int(x) for x in parts[1:11]]
    keys = ["user", "nice", "system", "idle", "iowait", "irq",
            "softirq", "steal", "guest", "guest_nice"]
    return dict(zip(keys, vals))


IRQ_MATCH = re.compile(r"xhci|enp8s0|enp0s31f6|eth|nvme|i915|nvidia", re.I)


def irq_counts():
    """/proc/interrupts 를 장치별로 합산. 이더넷/USB 인터럽트 부하 추적용."""
    out = {}
    try:
        with open("/proc/interrupts") as f:
            f.readline()
            for line in f:
                cols = line.split()
                if len(cols) < 3:
                    continue
                name = " ".join(cols[-1:])
                tail = " ".join(cols[1:])
                if not IRQ_MATCH.search(tail):
                    continue
                nums = []
                for c in cols[1:]:
                    if c.isdigit():
                        nums.append(int(c))
                    else:
                        break
                key = re.sub(r"[^A-Za-z0-9_]+", "_", name)[:24]
                out[key] = out.get(key, 0) + sum(nums)
    except Exception:
        pass
    return out


def nic_bytes():
    d = {}
    for n in NICS:
        for k in ("rx", "tx"):
            try:
                with open("/sys/class/net/%s/statistics/%s_bytes" % (n, k)) as f:
                    d[(n, k)] = int(f.read())
            except Exception:
                d[(n, k)] = 0
    return d


def main():
    os.makedirs(OUT, exist_ok=True)
    rospy.init_node("cr_monitor", anonymous=True, disable_signals=True)
    stats = [TopicStat(t) for t in TOPICS]

    f_proc = open(os.path.join(OUT, "proc.csv"), "w", newline="")
    w_proc = csv.writer(f_proc)
    w_proc.writerow(["ts", "label", "pids", "cpu_pct", "cpu_user_pct", "cpu_sys_pct",
                     "rss_mb", "threads", "read_mbps", "write_mbps", "gpu_mem_mb"])
    f_sys = open(os.path.join(OUT, "system.csv"), "w", newline="")
    w_sys = csv.writer(f_sys)
    w_sys.writerow(["ts", "cpu_mean_pct", "cpu_user_pct", "cpu_system_pct",
                    "cpu_iowait_pct", "cpu_irq_pct", "cpu_softirq_pct",
                    "mem_used_mb", "gpu_util_pct", "gpu_mem_mb"]
                   + ["rx_mbps_" + n for n in NICS] + ["tx_mbps_" + n for n in NICS])
    f_irq = open(os.path.join(OUT, "irq.csv"), "w", newline="")
    w_irq = csv.writer(f_irq)
    w_irq.writerow(["ts", "device", "irqs_per_s"])
    f_top = open(os.path.join(OUT, "topics.csv"), "w", newline="")
    w_top = csv.writer(f_top)
    w_top.writerow(["ts", "topic", "hz", "mbps", "drops", "cum_msgs", "cum_drops",
                    "lat_avg_ms", "lat_max_ms", "resync", "seq_valid"])

    io_prev = {}      # pid -> (read, write)
    psutil.cpu_percent(None)
    for procs in scan_procs().values():
        for p in procs:
            try: p.cpu_percent(None)
            except Exception: pass
    nic_prev = nic_bytes()
    stat_prev = cpu_stat()
    irq_prev = irq_counts()
    cput_prev = {}   # pid -> (user, system)

    t0 = time.time()
    t_prev = t0
    rospy.loginfo("[cr_monitor] 시작 -> %s (주기 %.1fs, %s)",
                  OUT, PERIOD, ("%.0f초" % DUR) if DUR else "Ctrl-C 까지")
    try:
        while not rospy.is_shutdown():
            time.sleep(PERIOD)
            now_t = time.time()
            # 표본 간 "실제" 경과. nvidia-smi 호출 등으로 PERIOD 보다 길어지므로
            # 모든 비율(Hz, Mbps, MB/s, CPU%)은 이 값으로 나눈다.
            dt = now_t - t_prev
            t_prev = now_t
            if dt <= 0:
                dt = PERIOD
            ts = round(now_t - t0, 2)

            cpu_tot = psutil.cpu_percent(None)
            vm = psutil.virtual_memory()
            (gu, gm), gpid = gpu_cached()
            nic_now = nic_bytes()
            stat_now = cpu_stat()
            irq_now = irq_counts()

            # /proc/stat 차분 -> 항목별 CPU 비율 (전 코어 합 기준 100%)
            dt_j = sum(stat_now[k] - stat_prev[k] for k in stat_now) or 1
            def jp(k):
                return round(100.0 * (stat_now[k] - stat_prev[k]) / dt_j, 2)
            w_sys.writerow([ts, round(cpu_tot, 1), jp("user"), jp("system"),
                            jp("iowait"), jp("irq"), jp("softirq"),
                            round(vm.used / 1e6, 1), gu, gm]
                           + [round((nic_now[(n, "rx")] - nic_prev[(n, "rx")]) * 8 / 1e6 / dt, 2) for n in NICS]
                           + [round((nic_now[(n, "tx")] - nic_prev[(n, "tx")]) * 8 / 1e6 / dt, 2) for n in NICS])
            for dev in sorted(set(irq_now) | set(irq_prev)):
                d = irq_now.get(dev, 0) - irq_prev.get(dev, 0)
                if d:
                    w_irq.writerow([ts, dev, round(d / dt, 1)])
            nic_prev = nic_now; stat_prev = stat_now; irq_prev = irq_now

            for lab, procs in sorted(scan_procs_cached().items()):
                cpu = rss = rd = wr = gmem = 0.0
                cu = cs = 0.0; thr = 0
                pids = []
                for p in procs:
                    try:
                        pids.append(p.pid)
                        cpu += p.cpu_percent(None)
                        try:
                            ct = p.cpu_times()
                            pu, ps_ = cput_prev.get(p.pid, (ct.user, ct.system))
                            cu += max(0.0, ct.user - pu) / dt * 100.0
                            cs += max(0.0, ct.system - ps_) / dt * 100.0
                            cput_prev[p.pid] = (ct.user, ct.system)
                        except Exception:
                            pass
                        try:
                            thr += p.num_threads()
                        except Exception:
                            pass
                        rss += p.memory_info().rss
                        try:
                            io = p.io_counters()
                            pr, pw = io_prev.get(p.pid, (io.read_bytes, io.write_bytes))
                            rd += max(0, io.read_bytes - pr)
                            wr += max(0, io.write_bytes - pw)
                            io_prev[p.pid] = (io.read_bytes, io.write_bytes)
                        except Exception:
                            pass
                        gmem += gpid.get(p.pid, 0.0)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                w_proc.writerow([ts, lab, "|".join(str(x) for x in pids),
                                 round(cpu, 1), round(cu, 1), round(cs, 1),
                                 round(rss / 1e6, 1), thr,
                                 round(rd / 1e6 / dt, 2), round(wr / 1e6 / dt, 2),
                                 round(gmem, 1)])

            for s in stats:
                n, b, d, lat, lmax, rs = s.take()
                w_top.writerow([ts, s.topic, round(n / dt, 2),
                                round(b * 8 / 1e6 / dt, 3), d, s.cum_n, s.cum_drop,
                                ("%.1f" % (lat * 1000)) if lat == lat else "",
                                ("%.1f" % (lmax * 1000)) if lmax else "",
                                rs, int(s.seq_ok)])

            f_proc.flush(); f_sys.flush(); f_top.flush(); f_irq.flush()
            if DUR and (time.time() - t0) >= DUR:
                break
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - t0
    with open(os.path.join(OUT, "summary.txt"), "w") as f:
        f.write("측정 시간: %.1f초\n\n" % elapsed)
        f.write("토픽별 누적\n")
        f.write("%-34s %10s %8s %8s %8s\n" % ("topic", "msgs", "avg_hz", "drops", "drop%"))
        for s in stats:
            dr = 100.0 * s.cum_drop / max(s.cum_n + s.cum_drop, 1)
            f.write("%-34s %10d %8.2f %8d %7.2f%s%s\n" % (
                s.topic, s.cum_n, s.cum_n / elapsed, s.cum_drop, dr,
                "" if s.seq_ok else "  (seq 신뢰불가)",
                ("  재동기화 %d회(발행노드 재시작 추정)" % s.cum_resync) if s.cum_resync else ""))
        f.write("\n주: bag_record 행의 cpu/rss/write 를 빼면 bag 제외 자원량이 된다.\n")
    print(open(os.path.join(OUT, "summary.txt")).read())
    rospy.loginfo("[cr_monitor] 종료. 결과: %s", OUT)


if __name__ == "__main__":
    main()
