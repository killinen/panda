#!/usr/bin/env python3
from collections import namedtuple
import os
import random
import time
from typing import Optional
from urllib.parse import urlparse
import requests

from panda import Panda
from panda.python import ALTERNATIVE_EXPERIENCE as ALT_EXP
from panda.tests.safety_replay.replay_drive import replay_drive
from tools.lib.logreader import LogReader  # pylint: disable=import-error

DEFAULT_BASE_URL = "https://commadataci.blob.core.windows.net/openpilotci/"
BASE_URL = os.environ.get("SAFETY_REPLAY_BASE_URL", DEFAULT_BASE_URL)

ReplayRoute = namedtuple("ReplayRoute", ("route", "safety_mode", "param", "alternative_experience"), defaults=(0, 0))

logs = [
  ReplayRoute("2425568437959f9d|2019-12-22--16-24-37.bz2", Panda.SAFETY_HONDA_NIDEC),       # HONDA.CIVIC (fcw presents: 0x1FA blocked as expected)
  ReplayRoute("38bfd238edecbcd7|2019-06-07--10-15-25.bz2", Panda.SAFETY_TOYOTA, 66),        # TOYOTA.PRIUS
  ReplayRoute("f89c604cf653e2bf|2018-09-29--13-46-50.bz2", Panda.SAFETY_GM),                # GM.VOLT
  ReplayRoute("6fb4948a7ebe670e|2019-11-12--00-35-53.bz2", Panda.SAFETY_CHRYSLER),          # CHRYSLER.PACIFICA_2018_HYBRID
  ReplayRoute("791340bc01ed993d|2019-04-08--10-26-00.bz2", Panda.SAFETY_SUBARU),            # SUBARU.IMPREZA
  ReplayRoute("76b83eb0245de90e|2020-03-05--19-16-05.bz2", Panda.SAFETY_VOLKSWAGEN_MQB),    # VOLKSWAGEN.GOLF (MK7)
  ReplayRoute("d12cd943127f267b|2020-03-27--15-57-18.bz2", Panda.SAFETY_VOLKSWAGEN_PQ),     # 2009 VW Passat R36 (B6), supporting OP port not yet upstreamed
  ReplayRoute("fbbfa6af821552b9|2020-03-03--08-09-43.bz2", Panda.SAFETY_NISSAN),            # NISSAN.XTRAIL
  ReplayRoute("5b7c365c50084530_2020-04-15--16-13-24--3--rlog.bz2", Panda.SAFETY_HYUNDAI),  # HYUNDAI.SONATA
  ReplayRoute("610ebb9faaad6b43|2020-06-13--15-28-36.bz2", Panda.SAFETY_HYUNDAI_LEGACY),    # HYUNDAI.IONIQ_EV_LTD
  ReplayRoute("5ab784f361e19b78_2020-06-08--16-30-41.bz2", Panda.SAFETY_SUBARU_LEGACY),     # SUBARU.OUTBACK
  ReplayRoute("bb50caf5f0945ab1|2021-06-19--17-20-18.bz2", Panda.SAFETY_TESLA),             # TESLA.AP2_MODELS
  ReplayRoute("bd6a637565e91581_2021-10-29--22-18-31--1--rlog.bz2", Panda.SAFETY_MAZDA),    # MAZDA.CX9_2021
  # HONDA.CIVIC_2022
  ReplayRoute("1a5d045d2c531a6d_2022-06-07--22-03-00--1--rlog.bz2", Panda.SAFETY_HONDA_BOSCH, Panda.FLAG_HONDA_RADARLESS, ALT_EXP.DISABLE_DISENGAGE_ON_GAS),
]

def _is_url(s: str) -> bool:
  return s.startswith("http://") or s.startswith("https://")

def _infer_safety_params(lr):
  for msg in lr:
    if msg.which() == 'carParams':
      cp = msg.carParams
      safety_cfg = cp.safetyConfigs[0]
      mode = safety_cfg.safetyModel.raw
      param = safety_cfg.safetyParam
      alt_exp = cp.alternativeExperience
      if hasattr(lr, "reset"):
        lr.reset()
      return mode, param, alt_exp
  raise Exception("carParams not found in log. Set safety mode/param manually.")

def _get_request_headers() -> dict:
  # Avoid proxies/servers applying a content-encoding on already-compressed .bz2 logs.
  return {"Accept-Encoding": "identity", "User-Agent": "panda-safety-replay"}

def _get_auth_header() -> str:
  return os.environ.get("SAFETY_REPLAY_AUTHORIZATION", "").strip()

def _auth_should_apply(route: str) -> bool:
  # Don't send user tokens to the default comma blob store: Azure treats an unexpected Authorization
  # header as a signed request and returns 403.
  if not _get_auth_header():
    return False
  if _is_url(route):
    return True
  return BASE_URL != DEFAULT_BASE_URL

def _get_request_headers_for_route(route: str) -> dict:
  headers = _get_request_headers()
  if _auth_should_apply(route):
    headers["Authorization"] = _get_auth_header()
  return headers

def _file_has_bzip2_magic(path: str) -> bool:
  try:
    with open(path, "rb") as f:
      return f.read(3) == b"BZh"
  except OSError:
    return False

def _download_attempts() -> int:
  # A couple environment variable names supported for compatibility.
  raw = os.environ.get("SAFETY_REPLAY_DOWNLOAD_ATTEMPTS", "").strip()
  if not raw:
    raw = os.environ.get("SAFETY_REPLAY_DOWNLOAD_RETRIES", "").strip()
  try:
    attempts = int(raw) if raw else 10
  except ValueError:
    attempts = 10
  return max(1, attempts)

def _download_timeouts() -> tuple:
  connect_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_CONNECT_TIMEOUT_SEC", "").strip()
  read_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_READ_TIMEOUT_SEC", "").strip()
  try:
    connect = float(connect_s) if connect_s else 15.0
  except ValueError:
    connect = 15.0
  try:
    read = float(read_s) if read_s else 60.0
  except ValueError:
    read = 60.0
  return (max(1.0, connect), max(1.0, read))

def _is_retryable_status(status_code: int) -> bool:
  return status_code in (408, 425, 429, 500, 502, 503, 504)

def _retry_sleep(attempt_index: int) -> None:
  base_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_BACKOFF_SEC", "").strip()
  max_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_BACKOFF_MAX_SEC", "").strip()
  try:
    base = float(base_s) if base_s else 0.5
  except ValueError:
    base = 0.5
  try:
    max_delay = float(max_s) if max_s else 10.0
  except ValueError:
    max_delay = 10.0

  delay = min(max(0.0, max_delay), max(0.0, base) * (2 ** max(0, attempt_index)))
  # Jitter to avoid synchronized retries in CI.
  delay *= random.uniform(0.7, 1.3)
  if delay > 0:
    time.sleep(delay)

def _download_if_needed(route: str, force: bool = False) -> str:
  if _is_url(route):
    url = route
    path = urlparse(url).path
    local = os.path.basename(path) if path else "safety_replay_log.bz2"
  else:
    url = BASE_URL + route
    local = route

  if os.path.isfile(local) and not force:
    # If a previous run cached a non-bzip2 response (e.g. HTML/JSON auth error), redownload.
    if _file_has_bzip2_magic(local):
      return local
    force = True

  if not os.path.isfile(local) or force:
    tmp = local + ".tmp"
    headers = _get_request_headers_for_route(route)
    timeouts = _download_timeouts()
    attempts = _download_attempts()
    last_exc: Optional[BaseException] = None

    for attempt in range(1, attempts + 1):
      if os.path.exists(tmp):
        os.remove(tmp)

      try:
        resp = requests.get(url, timeout=timeouts, headers=headers, stream=True)
        try:
          if resp.status_code >= 400:
            if _is_retryable_status(resp.status_code) and attempt < attempts:
              print(f"download failed (status={resp.status_code}) for {url}; retrying ({attempt}/{attempts})")
              _retry_sleep(attempt - 1)
              continue
            resp.raise_for_status()

          it = resp.iter_content(chunk_size=64 * 1024)
          first = next(it, b"")
          if len(first) < 3 or first[:3] != b"BZh":
            preview = first[:200]
            try:
              preview_text = preview.decode("utf-8", errors="replace")
            except Exception:
              preview_text = repr(preview)

            # Retry unexpected server-side responses (e.g. transient HTML errors); don't loop on auth errors.
            if _is_retryable_status(resp.status_code) and attempt < attempts:
              print(f"download returned non-bzip2 content for {url}; retrying ({attempt}/{attempts})")
              _retry_sleep(attempt - 1)
              continue
            raise Exception(
              f"downloaded non-bzip2 content from {url} "
              f"(status={resp.status_code}, auth_header={'set' if 'Authorization' in headers else 'unset'}, "
              f"content-type={resp.headers.get('Content-Type')}, "
              f"content-encoding={resp.headers.get('Content-Encoding')}, preview={preview_text!r})"
            )

          with open(tmp, "wb") as f:
            f.write(first)
            for chunk in it:
              if chunk:
                f.write(chunk)

          os.replace(tmp, local)
          break
        finally:
          resp.close()
      except (requests.exceptions.ConnectionError,
              requests.exceptions.Timeout,
              requests.exceptions.ChunkedEncodingError) as e:
        last_exc = e
        if attempt < attempts:
          print(f"download error for {url}: {type(e).__name__}: {e}; retrying ({attempt}/{attempts})")
          _retry_sleep(attempt - 1)
          continue
        raise
      except Exception as e:
        last_exc = e
        raise
      finally:
        if os.path.exists(tmp):
          os.remove(tmp)

    if not os.path.isfile(local):
      raise Exception(f"failed to download {url} after {attempts} attempts: {last_exc!r}")
  return local


if __name__ == "__main__":
  # Optional custom log, configured via CI env/vars.
  custom_url = os.environ.get("SAFETY_REPLAY_CUSTOM_URL", "").strip()
  if custom_url:
    print(f"adding custom safety replay log: {custom_url}")
    print(f"custom safety replay auth header: {'set' if _get_auth_header() else 'unset'}")
    mode_s = os.environ.get("SAFETY_REPLAY_CUSTOM_SAFETY_MODE", "").strip()
    param_s = os.environ.get("SAFETY_REPLAY_CUSTOM_SAFETY_PARAM", "").strip() or "0"
    alt_s = os.environ.get("SAFETY_REPLAY_CUSTOM_ALT_EXP", "").strip() or "0"
    print(f"custom safety replay params: mode={mode_s or '<unset>'}, param={param_s}, alt_exp={alt_s}")

    if mode_s.lower() == "auto":
      logs.append(ReplayRoute(custom_url, -1, -1, -1))
    else:
      if not mode_s:
        raise Exception("SAFETY_REPLAY_CUSTOM_SAFETY_MODE must be set (or 'auto') when SAFETY_REPLAY_CUSTOM_URL is set")
      logs.append(ReplayRoute(custom_url, int(mode_s), int(param_s), int(alt_s)))
  elif "SAFETY_REPLAY_CUSTOM_URL" in os.environ:
    print("SAFETY_REPLAY_CUSTOM_URL is set but empty; skipping custom safety replay log")

  # get all the routes
  for route, _, _, _ in logs:
    _download_if_needed(route)

  failed = []
  for route, mode, param, alt_exp in logs:
    local = _download_if_needed(route)
    try:
      lr = LogReader(local)
    except OSError:
      # Corrupt/truncated cache; redownload once.
      local = _download_if_needed(route, force=True)
      lr = LogReader(local)
    if mode < 0 or param < 0 or alt_exp < 0:
      mode, param, alt_exp = _infer_safety_params(lr)

    print("\nreplaying %s with safety mode %d, param %s, alternative experience %s" % (route, mode, param, alt_exp))
    if not replay_drive(lr, mode, param, alt_exp):
      failed.append(route)

    for f in failed:  # type: ignore
      print(f"\n**** failed on {f} ****")
    assert len(failed) == 0, "\nfailed on %d logs" % len(failed)
