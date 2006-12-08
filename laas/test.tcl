connect localhost
lm sfl

sfl::SetPOM xcfwrapOdometry
sfl::AddScanner xcfwrapScan 0.2  0.0  0.0  181  8.0  -1.5708  3.1416
sfl::SetModel 0.02  0.521  0.088  6.5  6.5  0.5  2.0  0.429  1.647
sfl::AddHullPoint   0.2   0.15
sfl::AddHullPoint  -0.2   0.15
sfl::AddHullPoint  -0.2  -0.15
sfl::AddHullPoint   0.2  -0.15
sfl::SetDWA  21  2.2  1.7  0.1  0.5  0.1  0.1
#sfl::SetBBand 4 8 4
sfl::SetGoalPoster xcfwrapGoal
sfl::SetCurspeed xcfwrapCurspeed

sfl::Start
