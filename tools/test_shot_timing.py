import unittest
from shot_timing import summarize


def shot(sid, seconds):
    return dict(event='shot_fired', kind='projectile', shot_id=sid,
                launch_timestamp_ns=int(seconds*1e9))


def hit(sid, seconds, team='blue'):
    return dict(event='armor_hit', kind='projectile', shot_id=sid,
                hit_timestamp_ns=int(seconds*1e9), team=team, armor_label=3)


class TimingTests(unittest.TestCase):
    def test_dedup_enemy_and_missing_launch(self):
        result = summarize([shot(1, 1), hit(1, 1.2), hit(1, 1.3),
                            shot(2, 2), hit(2, 2.2, 'red'), hit(99, 3)])
        self.assertEqual(result['unique_enemy_hits'], 1)
        self.assertIsNone(result['best_rolling_ttk'])

    def test_misses_and_inflight_rounds(self):
        events = [shot(1, 0), hit(1, 5), shot(2, 1),
                  shot(3, 2), hit(3, 2.2), shot(4, 3), hit(4, 3.2)]
        result = summarize(events, hp=40)
        self.assertAlmostEqual(result['first_burst_ttk']['seconds'], 3.2)
        self.assertEqual(result['best_rolling_ttk']['start_shot_id'], 3)
        self.assertAlmostEqual(result['best_rolling_ttk']['seconds'], 1.2)

    def test_round_up_and_invalid_inputs(self):
        self.assertEqual(summarize([], 150)['hits_required'], 8)
        for hp, damage in [(0, 20), (200, -1), (float('nan'), 20)]:
            with self.assertRaises(ValueError):
                summarize([], hp, damage)

    def test_fire_gate_categories_are_exclusive(self):
        events = [dict(event='sp_fire_gate', requested=r, aligned=a, rate_allowed=f)
                  for r, a, f in [(False, False, False), (True, False, False),
                                  (True, True, False), (True, True, True)]]
        self.assertEqual(summarize(events)['fire_gates'], {
            'valid_plans': 4, 'blocked_planner': 1, 'blocked_alignment': 1,
            'blocked_rate': 1, 'allowed': 1})


if __name__ == '__main__':
    unittest.main()
