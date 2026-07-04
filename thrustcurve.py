"""ThrustCurve.org API client (pure stdlib).

Lets RocketSim search the full ThrustCurve.org motor catalog, download a
motor's thrust curve, and save it into the local motors/ folder in the exact
CSV format the simulator expects.

Only the Python standard library is used (urllib.request, json, ssl) so no
extra pip dependencies are required. See https://www.thrustcurve.org/info/api.html
"""

import os
import re
import json
import ssl
import urllib.request
import urllib.error

# API base, overridable at CALL TIME (not import time) via env var so tests can
# point it at a local mock server.
API_BASE = "https://www.thrustcurve.org/api/v1"

_TIMEOUT = 10  # seconds
_USER_AGENT = "RocketSim/1.0"


class ThrustCurveError(Exception):
    """Raised for any network / HTTP / JSON / data failure talking to the API."""


def _base_url():
    """Resolve the API base URL, honouring THRUSTCURVE_API_BASE at call time."""
    return os.environ.get("THRUSTCURVE_API_BASE", API_BASE).rstrip("/")


def _post_json(endpoint, payload):
    """POST a JSON body to <base>/<endpoint> and return the parsed JSON dict.

    Raises ThrustCurveError with a human-readable message on any failure.
    """
    url = "{}/{}".format(_base_url(), endpoint.lstrip("/"))
    try:
        data = json.dumps(payload).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ThrustCurveError("Could not encode request: {}".format(exc))

    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    req.add_header("Accept", "application/json")
    req.add_header("User-Agent", _USER_AGENT)

    # Default SSL context (verifies certs). Kept as a variable so behaviour is
    # explicit; we never disable verification.
    context = ssl.create_default_context()

    try:
        with urllib.request.urlopen(req, timeout=_TIMEOUT, context=context) as resp:
            raw = resp.read()
    except urllib.error.HTTPError as exc:
        raise ThrustCurveError(
            "ThrustCurve.org returned HTTP {} for {}.".format(exc.code, endpoint)
        )
    except urllib.error.URLError as exc:
        raise ThrustCurveError(
            "Could not reach ThrustCurve.org ({}). "
            "Check your internet connection.".format(exc.reason)
        )
    except (ssl.SSLError, OSError) as exc:
        raise ThrustCurveError("Network error contacting ThrustCurve.org: {}".format(exc))

    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        text = raw.decode("utf-8", "replace")
    try:
        parsed = json.loads(text)
    except ValueError as exc:
        raise ThrustCurveError("ThrustCurve.org sent an unreadable response: {}".format(exc))
    if not isinstance(parsed, dict):
        raise ThrustCurveError("ThrustCurve.org sent an unexpected response format.")
    return parsed


def _num(value, default=0.0):
    """Best-effort float conversion; returns default on None / bad values."""
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _str(value, default=""):
    """Best-effort string conversion; returns default on None."""
    if value is None:
        return default
    if isinstance(value, str):
        return value
    return str(value)


def _normalize_motor(raw):
    """Normalize one raw API motor record into RocketSim's flat dict shape."""
    if not isinstance(raw, dict):
        raw = {}
    manufacturer = _str(raw.get("manufacturerAbbrev") or raw.get("manufacturer"))
    common_name = _str(raw.get("commonName"))
    designation = _str(raw.get("designation"))
    # Human-readable label: manufacturer + the most recognizable motor code.
    label_code = common_name or designation
    name = (manufacturer + " " + label_code).strip() or label_code or "Unknown motor"
    return {
        "motor_id": _str(raw.get("motorId")),
        "name": name,
        "manufacturer": _str(raw.get("manufacturer") or raw.get("manufacturerAbbrev")),
        "designation": designation,
        "common_name": common_name,
        "impulse_class": _str(raw.get("impulseClass")),
        "diameter_mm": _num(raw.get("diameter")),
        "length_mm": _num(raw.get("length")),
        "type": _str(raw.get("type")),
        "avg_thrust_n": _num(raw.get("avgThrustN")),
        "max_thrust_n": _num(raw.get("maxThrustN")),
        "total_impulse_ns": _num(raw.get("totImpulseNs")),
        "burn_time_s": _num(raw.get("burnTimeS")),
        "total_weight_g": _num(raw.get("totalWeightG")),
        "prop_weight_g": _num(raw.get("propWeightG")),
        "info_url": _str(raw.get("infoUrl")),
    }


def search_motors(common_name=None, manufacturer=None, impulse_class=None,
                  max_results=50):
    """Search the ThrustCurve.org catalog.

    Returns a list of normalized motor dicts (see _normalize_motor). Only
    criteria that are provided (truthy) are sent to the API.
    """
    criteria = {"maxResults": int(max_results)}
    if common_name:
        criteria["commonName"] = str(common_name)
    if manufacturer:
        criteria["manufacturer"] = str(manufacturer)
    if impulse_class:
        criteria["impulseClass"] = str(impulse_class)

    parsed = _post_json("search.json", criteria)
    results = parsed.get("results")
    if not isinstance(results, list):
        return []
    return [_normalize_motor(r) for r in results]


def download_samples(motor_id):
    """Download the thrust-curve samples for a motor id.

    Returns a list of (time, thrust) tuples sorted by time. Picks the result
    entry with the most samples. Ensures the curve starts at t=0. Raises
    ThrustCurveError if no samples are available.
    """
    motor_id = _str(motor_id)
    if not motor_id:
        raise ThrustCurveError("No motor id supplied for download.")

    parsed = _post_json("download.json", {"motorIds": [motor_id], "data": "samples"})
    results = parsed.get("results")
    if not isinstance(results, list) or not results:
        raise ThrustCurveError("No thrust-curve data returned for this motor.")

    best = None
    best_len = -1
    for entry in results:
        if not isinstance(entry, dict):
            continue
        samples = entry.get("samples")
        if not isinstance(samples, list):
            continue
        if len(samples) > best_len:
            best_len = len(samples)
            best = samples

    if not best:
        raise ThrustCurveError("This motor has no downloadable thrust-curve samples.")

    pairs = []
    for s in best:
        if not isinstance(s, dict):
            continue
        t = s.get("time")
        thr = s.get("thrust")
        if t is None or thr is None:
            continue
        try:
            pairs.append((float(t), float(thr)))
        except (TypeError, ValueError):
            continue

    if not pairs:
        raise ThrustCurveError("This motor has no usable thrust-curve samples.")

    pairs.sort(key=lambda p: p[0])

    # Ensure the curve starts at (0, 0) so integration/plotting behaves.
    first_t, first_thr = pairs[0]
    if first_t > 0 and first_thr > 0:
        pairs.insert(0, (0.0, 0.0))

    return pairs


def _sanitize_filename(name):
    """Turn a motor name into a safe CSV basename ([A-Za-z0-9._-], spaces -> _)."""
    name = _str(name).strip()
    name = name.replace(" ", "_")
    name = re.sub(r"[^A-Za-z0-9._-]", "", name)
    # Guard against empties or names that are all dots (path traversal, hidden).
    name = name.strip(".")
    if not name:
        name = "motor"
    return name + ".csv"


def save_motor_csv(motor, samples, motors_dir):
    """Write a motor's thrust curve as a local-library CSV.

    Layout (matches motors/AeroTech_F40W.csv exactly):
        "motor:","<Manufacturer> <CommonName>"
        "contributor:","ThrustCurve.org API"
        "details:","<info_url>"
        (empty line)
        "Time (s)","Thrust (N)"
        t,thrust
        ...

    Returns the bare filename (not the full path). Overwrites an existing file
    of the same name.
    """
    if not isinstance(motor, dict):
        motor = {}
    manufacturer = _str(motor.get("manufacturer"))
    common_name = _str(motor.get("common_name")) or _str(motor.get("designation"))
    display = (manufacturer + " " + common_name).strip() or _str(motor.get("name")) \
        or "ThrustCurve Motor"
    info_url = _str(motor.get("info_url"))

    filename = _sanitize_filename(motor.get("name") or display)
    # Extra path-traversal guard: never allow directory components.
    filename = os.path.basename(filename)
    os.makedirs(motors_dir, exist_ok=True)
    path = os.path.join(motors_dir, filename)

    lines = []
    lines.append('"motor:","{}"'.format(display))
    lines.append('"contributor:","ThrustCurve.org API"')
    lines.append('"details:","{}"'.format(info_url))
    lines.append("")
    lines.append('"Time (s)","Thrust (N)"')
    for t, thr in samples:
        lines.append("{},{}".format(_fmt_num(t), _fmt_num(thr)))

    with open(path, "w", newline="") as f:
        f.write("\n".join(lines) + "\n")

    return filename


def _fmt_num(x):
    """Format a number without trailing scientific noise; integers stay clean."""
    f = float(x)
    if f == int(f):
        return str(int(f))
    return repr(f)
