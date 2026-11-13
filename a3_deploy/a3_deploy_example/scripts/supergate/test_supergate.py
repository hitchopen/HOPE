import collections
import json
from pathlib import Path
import tempfile
import unittest
from common import HERE, REPOSITORY, load_dataset, summarize, sha256
from build import offline_clock_header
from freeze_dataset import freeze


class DatasetTests(unittest.TestCase):
    def test_frozen_suites_and_factorial_coverage(self):
        self.assertEqual(load_dataset(HERE/'data/main').shape,(6,200,11))
        self.assertEqual(load_dataset(HERE/'data/slow-tail').shape,(6,9,11))
        counts = collections.Counter()
        identities = set()
        for shard in range(1,7):
            rows = json.loads((HERE/f'data/main/shard-{shard:02d}-ledger.json').read_text())
            for row in rows:
                if row['section']=='factorial81':
                    counts[row['cell_id']]+=1
                identities.add(tuple(row['launch']+row['initial_spin']))
        self.assertEqual(len(identities),1200)
        self.assertEqual(len(counts),81)
        self.assertEqual(set(counts.values()),{12})

    def test_outer_and_mixed_long_original_suites(self):
        self.assertEqual(load_dataset(HERE/'data/outer').shape, (6,67,11))
        self.assertEqual(load_dataset(HERE/'data/mixed-long').shape, (6,200,11))
        suites = {}
        for label, count in [('outer',402), ('mixed-long',1200)]:
            root = HERE/'data'/label
            rows = [row for shard in range(1,7)
                    for row in json.loads((root/f'shard-{shard:02d}-ledger.json').read_text())]
            self.assertEqual(rows, json.loads((root/'samples.json').read_text()))
            self.assertEqual(len({tuple(r['launch']+r['initial_spin']) for r in rows}), count)
            suites[label] = rows
        cells = collections.Counter(r['cell_id'] for r in suites['outer'])
        self.assertEqual(len(cells), 67)
        self.assertEqual(set(cells.values()), {6})
        self.assertEqual(collections.Counter(r['validation_source'] for r in suites['mixed-long']),
                         {'outer':402, 'fresh_holdout':486, 'main':312})
        outer = {tuple(r['launch']+r['initial_spin']) for r in suites['outer']}
        mixed_outer = {tuple(r['launch']+r['initial_spin']) for r in suites['mixed-long']
                       if r['validation_source']=='outer'}
        self.assertEqual(outer, mixed_outer)

    def test_original_source_and_data_preservation(self):
        manifest = json.loads((HERE/'original/SOURCE_MANIFEST.json').read_text())
        for name, record in manifest['files'].items():
            self.assertEqual(sha256(HERE/name), record['sha256'], name)

    def test_pinned_planner_source_manifest(self):
        root = HERE/'native/planner'
        manifest = json.loads((root/'SOURCE_MANIFEST.json').read_text())
        for name, expected in manifest['sha256'].items():
            self.assertEqual(sha256(root/name), expected, name)

    def test_prefix_bounds(self):
        self.assertEqual(load_dataset(HERE/'data/main',1).shape,(6,1,11))
        for count in [0,-1,201]:
            with self.assertRaises(ValueError):
                load_dataset(HERE/'data/main',count)

    def test_freeze_is_reproducible_and_tampering_rejected(self):
        samples = [{'launch':[2.,i*.01,.5,-3.,0.,1.],'initial_spin':[0.,0.,0.],'flight_duration_s':2.} for i in range(12)]
        with tempfile.TemporaryDirectory() as temp:
            a,b=Path(temp)/'a',Path(temp)/'b'
            freeze(samples,a,17)
            freeze(samples,b,17)
            self.assertEqual((a/'SHA256.json').read_bytes(),(b/'SHA256.json').read_bytes())
            self.assertEqual(load_dataset(a).shape,(6,2,11))
            (a/'shard-01.yaml').write_text('tampered')
            with self.assertRaisesRegex(ValueError,'integrity'):
                load_dataset(a)

    def test_clock_overlay_uses_simulation_time_without_mutating_source(self):
        path=REPOSITORY/'a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_planner_input.hpp'
        source=path.read_text()
        generated=offline_clock_header(source)
        self.assertIn('inline thread_local double simulation_time_s',generated)
        self.assertIn('return 1800000000.0 + simulation_time_s',generated)
        self.assertIn('return 1000.0 + simulation_time_s',generated)
        self.assertEqual(path.read_text(),source)
        with self.assertRaises(ValueError):
            offline_clock_header('changed unsupported clock API')

    def test_missing_results_cannot_be_reported_as_success(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(FileNotFoundError):
                summarize(temp,['candidate'],1)


if __name__ == '__main__':
    unittest.main()
