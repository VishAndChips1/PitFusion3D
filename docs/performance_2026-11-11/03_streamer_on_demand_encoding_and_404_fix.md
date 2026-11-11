# 03. Web streamer: encode on demand, and an HTTP bug

## Problem
`fusion_web_streamer` quantized and gzipped every cloud at 10 Hz (live) and ~1.5 Hz (merged) whether or not anyone was watching: **26% of a core** after the transport fix ([`shm.json`](evidence/measurements/shm.json)), measured with no viewer connected.

## Changes (`fusion_web_streamer.py`)
1. **Lazy encode:** the ROS callback only remembers the newest cloud; the HTTP handler encodes it, at most once per cloud, only when a client asks for newer data (ETag/304 behaviour unchanged). Idle cost ~0; always serves the newest data.
2. **404 bug (pre-existing):** unknown paths returned no `Content-Length` under HTTP/1.1 keep-alive, so the client waited for a body forever. Every browser requests `/favicon.ico`, so each page load hung one connection until the 30 s timeout. Now `Content-Length: 0`.
3. Routine client disconnects (flaky WiFi, closed tab) no longer print a traceback each (45 of them in one test run).

## Before / after
| | streamer CPU |
|---|---|
| after transport fix, no viewer (always encoding) | 26% |
| on-demand, no viewer | 6% |
| on-demand, 1 viewer polling like the real page | 20% (8-min soak) |

## Corner cases (15/15 pass, [output](evidence/corner_cases/streamer_suite_AFTER.txt))
Request before any data returns a valid empty payload; capped at 20,000 / 5,000 points; same ETag -> 304; after 30 s idle the first request returns the newest data; 16 concurrent clients, 3,364 requests, 0 errors (streamer ~66% of a core under that
abuse); 60 mid-response disconnects; garbage / 70 KB-header / POST requests; zero-extent, all-NaN-thermal and empty clouds; recovers afterwards. The 404 hang is visible in the
[before output](evidence/corner_cases/streamer_suite_BEFORE_404_fix.txt) (`TimeoutError` on `/nonexistent`).
