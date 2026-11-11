import http.client, sys, time, threading
dur = float(sys.argv[1]); end = time.time() + dur
def poll(path, iv):
    c = http.client.HTTPConnection('localhost', 8080, timeout=4); etag = None
    while time.time() < end:
        t = time.time()
        try:
            h = {'If-None-Match': etag} if etag else {}
            c.request('GET', path, headers=h); r = c.getresponse(); r.read()
            etag = r.getheader('ETag') or etag
        except Exception:
            c = http.client.HTTPConnection('localhost', 8080, timeout=4)
        time.sleep(max(0.02, iv - (time.time() - t)))
for p, iv in (('/merged.bin', 0.25), ('/live.bin', 0.12)):
    threading.Thread(target=poll, args=(p, iv), daemon=True).start()
time.sleep(dur)
