

class Note():

    def __init__(self, note : str, start, duration, finger : str):
        self.note = note
        self.start = start
        self.duration = duration
        self.finger = finger

    def __repr__(self):
        return f"\n{self.note} at t={self.start} for {self.duration} seconds on finger {self.finger}"



NOTE_DUR = 0.25 * 1.5
INTER_NOTE_DUR = 0.03 * 5

R = "R"
L = "L"

l_theme_mappings = [
        ("F", "R"),
        ("F", "R"),
        ("D", "L"),
        ("G", "R"),
        ("D", "L"),
        ("E", "L"),
        ("F", "R"),
        ("D", "L"),
        ("A", "R"),
        ("G", "R"),
        ("F", "R"),
        ("E", "L"),
        ("D", "L"),
        ("C", "L"),
        ("D", "L"),
        ("F", "R"),
        ("D", "L"),
        ("G", "R"),
        ("D", "L"),
        ("E", "L"),
        ("F", "R"),
        ("D", "L"),
        ("A", "R"),
        ("G", "R"),
        ("F", "R"),
        ("E", "L"),
        ("D", "L"),
        ("C", "L"),
        ("D", "R")
        ]

l_theme = []
for i, (note, finger) in enumerate(l_theme_mappings):
    l_theme.append(Note(note, (NOTE_DUR + INTER_NOTE_DUR) * i, NOTE_DUR, finger))



little_lamb_mappings = [
        ("E", "R"),
        ("D", "L"),
        ("C", "L"),
        ("D", "R"),
        ("E", "R"),
        ("E", "R"),
        ("E", "R"),
        ("D", "L"),
        ("D", "L"),
        ("D", "L"),
        ("E", "L"),
        ("G", "R"),
        ("G", "R"),
        ("E", "R"),
        ("D", "R"),
        ("C", "L"),
        ("D", "L"),
        ("E", "R"),
        ("E", "R"),
        ("E", "R"),
        ("E", "R"),
        ("D", "L"),
        ("D", "L"),
        ("E", "R"),
        ("D", "R"),
        ("C", "L")
        ]

little_lamb = []
for i, (note, finger) in enumerate(little_lamb_mappings):
    little_lamb.append(Note(note, (NOTE_DUR + INTER_NOTE_DUR) * i, NOTE_DUR, finger))


EIGTH = NOTE_DUR / 2
HALF = NOTE_DUR * 2
DOTTED_QUARTER = NOTE_DUR * 1.5
QUARTER = NOTE_DUR
ode_to_joy_mappings = [
    ("E", "R", QUARTER),
    ("E", "L", QUARTER),
    ("F", "R", QUARTER),
    ("G", "R", QUARTER),
    ("G", "R", QUARTER),
    ("F", "R", QUARTER),
    ("E", "R", QUARTER),
    ("D", "R", QUARTER),
    ("C", "R", QUARTER),
    ("C", "R", QUARTER),
    ("D", "R", QUARTER),
    ("E", "R", QUARTER),
    ("E", "R", DOTTED_QUARTER),
    ("D", "R", EIGTH),
    ("D", "R", HALF),

    ("E", "R", QUARTER),
    ("E", "R", QUARTER),
    ("F", "R", QUARTER),
    ("G", "R", QUARTER),
    ("G", "R", QUARTER),
    ("F", "R", QUARTER),
    ("E", "R", QUARTER),
    ("D", "R", QUARTER),
    ("C", "R", QUARTER),
    ("C", "R", QUARTER),
    ("D", "R", QUARTER),
    ("E", "R", QUARTER),
    ("D", "R", DOTTED_QUARTER),
    ("C", "R", EIGTH),
    ("C", "R", HALF),


    ("D", "R", QUARTER),
    ("D", "R", QUARTER),
    ("E", "R", QUARTER),
    ("C", "R", QUARTER),
    ("D", "R", QUARTER),
    ("E", "R", EIGTH),
    ("F", "R", EIGTH),
    ("E", "R", QUARTER),
    ("C", "R", QUARTER),
    ("D", "R", QUARTER),
    ("E", "R", EIGTH),
    ("F", "R", EIGTH),
    ("E", "R", QUARTER),
    ("D", "R", QUARTER),
    ("C", "R", QUARTER),
    ("D", "L", QUARTER),
    ("G", "R", HALF)
    ]

ode_to_joy = []
for i, (note, finger, duration) in enumerate(ode_to_joy_mappings):
    if i == 0:
        ode_to_joy.append(Note(note, 0, duration, finger))
    else:
        ode_to_joy.append(Note(note, INTER_NOTE_DUR + ode_to_joy[-1].start + ode_to_joy[-1].duration, duration, finger))

ode_to_joy.insert(0, Note("C", 0, NOTE_DUR, "L"))


if __name__ == "__main__":
    print("Song")
    #print(l_theme)
    print(ode_to_joy)



