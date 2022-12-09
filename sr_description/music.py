

class Note():

    def __init__(self, note : str, start, duration, finger : str):
        self.note = note
        self.start = start
        self.duration = duration
        self.finger = finger

    def __repr__(self):
        return f"\n{self.note} at t={self.start} for {self.duration} seconds on finger {self.finger}"



NOTE_DUR = 0.25
INTER_NOTE_DUR = 0.03

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


if __name__ == "__main__":
    print("Song")
    print(l_theme)




