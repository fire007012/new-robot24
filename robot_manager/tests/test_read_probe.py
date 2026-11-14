from concurrent.futures import Future
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from read_probe import ReadProbe


class Tests(unittest.TestCase):
    def test_lost_first_read_is_retried_without_promoting_old_or_missing_data(self):
        class Client:
            def __init__(self): self.futures=[]
            def service_is_ready(self): return True
            def call_async(self,request):
                future=Future(); self.futures.append(future); return future
        now=[0.0];client=Client();probe=ReadProbe(client,lambda:object(),lambda:now[0])
        self.assertIsNone(probe.poll()); self.assertEqual(len(client.futures),1)
        now[0]=.9;self.assertIsNone(probe.poll());self.assertEqual(len(client.futures),1)
        now[0]=1.01;self.assertIsNone(probe.poll());self.assertTrue(client.futures[0].cancelled())
        self.assertEqual(len(client.futures),2)
        client.futures[1].set_result({'controllers':['inactive']})
        self.assertEqual(probe.poll(),{'controllers':['inactive']})

    def test_unavailable_or_failed_read_never_counts_as_healthy(self):
        class Client:
            ready=False
            def service_is_ready(self):return self.ready
            def call_async(self,request):
                future=Future();future.set_exception(RuntimeError('lost response'));return future
        client=Client();probe=ReadProbe(client,lambda:None)
        self.assertIsNone(probe.poll());self.assertIsNone(probe.future)
        client.ready=True;self.assertIsNone(probe.poll());self.assertIsNone(probe.poll())

if __name__=='__main__':unittest.main()
