# Master Note — Adaptive PID / Learning-Based Tuning
Last updated: 2025-12-27

## 1. Real-World Problem (Ground Truth)
(Write this from lived experience, not papers.)

- Where does classical PID break in practice?
- What tuning pain did I personally face? (e.g., grade, load, friction)
- Why is this hard to solve with rules alone?

---

## 2. Core Question (The One That Matters)
(One sentence. Ruthlessly narrow.)

> Can learning reduce manual PID tuning effort on real vehicles by providing
> better initial gains or bounded adaptation using response history alone?

---

## 3. Non-Negotiable Constraints
(These define your credibility.)

- No oracle parameters (friction, grade, mass)
- PID remains in the fast loop
- Learning influence is bounded and slow
- Deployable mindset (timing, safety, validation)

---

## 4. Working Hypotheses (Expected to Change)
(Keep these tentative. Update aggressively.)

- H1: Most adaptive PID papers optimize tracking performance, not tuning effort.
- H2: Many methods assume parameters unavailable on real vehicles.
- H3: Response-history-only signals may be sufficient for bounded gain adaptation.
- H4: Learning helps reduce tuning iterations, not eliminate tuning.

---

## 5. What I’m Actively Looking For in Literature
(Use this to read *against* papers.)

- Does the method assume known friction/grade/mass?
- Is learning replacing control or supervising it?
- Is evaluation vehicle-like or toy?
- Is tuning effort discussed explicitly?

---

## 6. Early Patterns Observed (Fill As You Go)
(Short bullets only.)

- 
- 
- 

---

## 7. Contradictions / Disconfirming Evidence
(Force honesty here.)

- 
- 
- 

---

## 8. What Would Make This Not Worth Pursuing?
(Stop conditions.)

- If response-history-only methods fail consistently
- If similar deployable approaches already exist and are mature
- If gains don’t generalize beyond narrow scenarios

---

## 9. Provisional Direction (Not a Commitment)
(This is allowed to be wrong.)

- Potential approach:
  - Offline learning for gain initialization
  - Online bounded adaptation as optional
- Evaluation focus:
  - Tuning iteration reduction
  - Recovery time after disturbances
  - Engineer-in-the-loop usability

---