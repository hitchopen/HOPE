"""Read-only layout keeps unknown runtime health distinct from absent data."""
import json
from pathlib import Path
import unittest


class ObserverLayoutStatusTests(unittest.TestCase):
    def test_visible_session_and_planner_panels_show_diagnostic_strings(self):
        path = Path(__file__).resolve().parents[1] / "layouts/model21800_observer.json"
        layout = json.loads(path.read_text())
        panels = layout["configById"]

        def visible(tree):
            if isinstance(tree, str):
                return {tree}
            return visible(tree["first"]) | visible(tree["second"])

        rendered = visible(layout["layout"])
        for panel, topic in (
            ("RawMessages!session", "/hope/session/text"),
            ("RawMessages!planner", "/hope/planner/process_status"),
            ("RawMessages!telemetryfaults", "/hope/operations/telemetry_fault"),
        ):
            with self.subTest(panel=panel):
                self.assertIn(panel, rendered)
                self.assertEqual(panels[panel]["topicPath"], topic)
                self.assertFalse(panels[panel]["diffEnabled"])
        # Restore colored summaries, but never translate unknown health into a
        # definite missing session/process. The prominent freshness panel tells
        # the operator when these Bool summaries cannot establish live state.
        for panel, topic in (
            ("Indicator!session", "/hope/session/active.data"),
            ("Indicator!planner", "/hope/planner/process_alive.data"),
        ):
            self.assertIn(panel, rendered)
            self.assertEqual(panels[panel]["path"], topic)
            rules = {rule["rawValue"]: rule for rule in panels[panel]["rules"]}
            self.assertNotEqual(rules["true"]["color"], rules["false"]["color"])
            self.assertIn("NOT CONFIRMED", rules["false"]["label"])
        freshness = panels["Indicator!runtimeknown"]
        self.assertEqual(freshness["path"], "/hope/session/runtime_known.data")
        false_rule = next(rule for rule in freshness["rules"] if rule["rawValue"] == "false")
        self.assertIn("UNKNOWN", false_rule["label"])
        self.assertIn("IGNORE SESSION / PLANNER", false_rule["label"])
        top_row = layout["layout"]["first"]["first"]
        self.assertEqual(top_row["second"], "Indicator!runtimeknown")
        self.assertLessEqual(top_row["splitPercentage"], 50)
        for panel in rendered:
            self.assertNotIn(panel.split("!", 1)[0], {"ServiceCall", "Publish"})


if __name__ == "__main__":
    unittest.main()
