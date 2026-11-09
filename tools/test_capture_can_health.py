import unittest
from capture_can_health import compare


class CounterDeltaTest(unittest.TestCase):
    def test_reports_interval_delta_instead_of_historical_total(self):
        before = {'ifindex': 7, 'stats64': {'tx': {'errors': 5}}, 'xstats': {'bus_off': 35}}
        after = {'ifindex': 7, 'stats64': {'tx': {'errors': 6}}, 'xstats': {'bus_off': 35}}
        result = compare(before, after)
        self.assertTrue(result['comparable'])
        self.assertEqual(result['delta'], {'stats64.tx.errors': 1, 'xstats.bus_off': 0})

    def test_recreated_interfaces_and_reset_counters_are_not_compared(self):
        before = {'ifindex': 7, 'xstats': {'bus_off': 35}}
        for after in ({'ifindex': 8, 'xstats': {'bus_off': 35}},
                      {'ifindex': 7, 'xstats': {'bus_off': 0}},
                      {'ifindex': 7}):
            self.assertFalse(compare(before, after)['comparable'])
