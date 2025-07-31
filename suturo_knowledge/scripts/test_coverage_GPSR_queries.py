import re
import os
from pathlib import Path

def extract_functions_from_markdown(md_text):
    # hole alle <summary>...</summary> Inhalte
    # ignoriere "unsorted debug queries" 
    summaries = re.findall(r"<summary>(.*?)</summary>", md_text, re.DOTALL)
    # betrachte nur die Namen, nicht Parameter
    functions = []
    for s in summaries:
        s = s.strip()
        if not s or 'debug' in s.lower():
            continue
        func_name = s.split('(')[0].strip()
        if func_name:
            functions.append(func_name)
    return functions

def extract_test_functions_from_code(code_text):
    tests = re.findall(r'def\s+(test_\w+)', code_text)
    return [fn.replace('test_', '') for fn in tests]

def compare_coverage(checklist, tests):
    print("Test Coverage Report:\n" + "=" * 25)
    for f in sorted(checklist):
        if f in tests:
            print(f". {f}")
        else:
            print(f"X {f}")
    covered = len([f for f in checklist if f in tests])
    print(f"\nCoverage: {covered}/{len(checklist)} functions covered ({covered / len(checklist) * 100:.1f}%)")

# ---- MAIN ----
md_path = os.path.expanduser("~/Downloads/GPSR Queries (Updated).md")

with open(md_path, "r", encoding="utf-8") as f:
    check_text = f.read()


test_path = Path("GPSR_query_tests.py")
test_text = test_path.read_text()

checklist = extract_functions_from_markdown(check_text)
tests = extract_test_functions_from_code(test_text)

compare_coverage(checklist, tests)
