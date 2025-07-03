# A script to generate pathplanner .auto files from a much more simple auto file.
"""
syntax:
    Command names are written without quotes. Nontrivial spaces (between regular characters) will be
    respected.
    To join commands in a sequential command group, use the + operator.
    To join commands in a parallel command group, use the & operator.
    To join commands in a race group, use the * operator
    To join commands in a deadline group, use the ? operator
    To make sure that any nesting is done properly, use parens to explicity
    define order of operations
    Multiple lines in a file are different commands running in sequence

    You can leave comments with the # sign. Anything after the # sign is ignored.
    Comments MUST BE ON THEIR OWN LINE

    Here's a command that runs A and B in parallel, then runs C:

        (A & B) + C

    This command runs A, B, and C in sequence, in a race group with D:

        (A + B + C) * D

    The outputted json is pathplanner-ready and can be saved to src/main/deploy/pathplanner/autos/<NAME>.auto
"""

from enum import Enum
from io import UnsupportedOperation
import sys

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} [filename]")
    sys.exit(1)

filename = sys.argv[1]

file = open(filename, "r")
text = list(map(lambda s: s.strip(), file.readlines()))
file.close()

class Token(Enum):
    OPEN_PAREN = "("
    CLOSE_PAREN = ")"
    PLUS = "+"
    AND = "&"
    DEADLINE = "?"
    RACE = "*"

    def __repr__(self):
        return f"('{self.value}')"

TOKENS = [Token.OPEN_PAREN.value, Token.CLOSE_PAREN.value, Token.PLUS.value, Token.AND.value, Token.DEADLINE.value, Token.RACE.value]

class Scanner:
    def __init__(self, source):
        self.start = 0
        self.current = 0
        self.tokens = []
        self.source = source

    def scan(self):
        while self.start < len(self.source):
            self.scan_token()
            self.start = self.current

    def scan_token(self):
        c = self.source[self.current]
        self.current += 1
        match c:
            case Token.OPEN_PAREN.value:
                self.tokens.append(Token.OPEN_PAREN)
            case Token.CLOSE_PAREN.value:
                self.tokens.append(Token.CLOSE_PAREN)
            case Token.PLUS.value:
                self.tokens.append(Token.PLUS)
            case Token.AND.value:
                self.tokens.append(Token.AND)
            case Token.DEADLINE.value:
                self.tokens.append(Token.DEADLINE)
            case Token.RACE.value:
                self.tokens.append(Token.RACE)
            case '#':
                self.comment()
            case _:
                # assume it's an identifier, so run ident
                self.ident()

    def comment(self):
        # ignore the rest of the thing
        self.current = len(self.source)

    def ident(self):
        while self.current < len(self.source) and self.source[self.current] not in TOKENS:
            self.current += 1
        # consume the last character
        s = self.source[self.start:self.current].strip()
        if s:
            self.tokens.append(Named(s))

class Command:
    def __init__(self, kind):
        self.kind = kind

    def andThen(self, other):
        if other.kind == Sequential.kind:
            return other.andThen(self)
        return Sequential([self, other])

    def alongWith(self, other):
        if other.kind == Parallel.kind:
            return other.alongWith(self)
        return Parallel([self, other])

    def deadlineFor(self, other):
        if other.kind == Parallel.kind:
            return Deadline([self] + other.inner)
        return Deadline([self, other])

    def raceWith(self, other):
        if other.kind == Race.kind:
            return other.raceWith(self)
        cmds = []
        if other.kind == Parallel.kind:
            cmds += other.inner
        else:
            cmds.append(other)
        cmds.append(self)
        return Race(cmds)

    def json(self):
        raise NotImplementedError("json() method not implemented on general class type")

    def __repr__(self):
        return f"<{self.kind}>";

class GroupCommand(Command):
    def __init__(self, cmds=[]):
        self.inner = cmds

    def json(self):
        return f'{{"type":"{self.kind}","data":{{"commands":[{",".join(map(lambda c: c.json(), self.inner))}]}}}}'

    def __repr__(self):
        return f"[{self.kind[:3].upper()} {','.join(map(lambda c: str(c), self.inner))}]"

class Sequential(GroupCommand):
    kind = "sequential"

    def andThen(self, other):
        if other.kind == Sequential.kind:
            self.inner += other.inner
            return self
        self.inner.append(other)
        return self

class Parallel(GroupCommand):
    kind = "parallel"

    def alongWith(self, other):
        if other.kind == Parallel.kind:
            self.inner += other.inner
            return self
        self.inner.append(other)
        return self

    def raceWith(self, other):
        if other.kind == Race.kind:
            return other.raceWith(self)
        if other.kind == Parallel.kind:
            return Race(self.inner + other.inner)
        return Race([other] + self.inner)

class Race(GroupCommand):
    kind = "race"

    def raceWith(self, other):
        if other.kind == Parallel.kind or other.kind == Race.kind:
            self.inner += other.inner
        self.inner.append(other)
        return self

class Deadline(GroupCommand):
    kind = "deadline"

    def deadlineFor(self, other):
        if other.kind == Deadline.kind:
            raise UnsupportedOperation("Deadline cannot have two deadlines")
        if other.kind == Parallel.kind:
            self.inner += other.inner
        else:
            self.inner.append(other)
        return self

class Named(Command):
    kind = "named"

    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return f"<{self.name}>"

    def json(self):
        return f'{{"type":"named","data":{{"name":"{self.name}"}}}}'

class Parser:
    def __init__(self, tokens):
        self.tokens = tokens
        self.current = 0
        self.tree = Sequential()

    def find_groups(self, tokens):
        # look for first parenthesis in the group.
        i = 0
        while i < len(tokens):
            if tokens[i] == Token.OPEN_PAREN:
                break;
            i += 1
        else:
            # never left the loop, no parenthesis found
            return self.condense(tokens)
        # we now know that a parenthesis is found at index i.
        j = i + 1
        offset = 1 # +1 for open, -1 for close
        while j < len(tokens):
            if tokens[j] == Token.OPEN_PAREN:
                offset += 1
            elif tokens[j] == Token.CLOSE_PAREN:
                offset -= 1
            if offset < 0:
                print("parenthesis parse error!")
                print(tokens)
                sys.exit(1)
            elif offset == 0:
                break
            j += 1
        else:
            print("did not close parenthesis!")
            print(tokens)
            sys.exit(1)
        inner = self.find_groups(tokens[i+1:j])
        return self.condense(tokens[:i] + inner + self.find_groups(tokens[j + 1:]))

    def condense(self, tokens):
        # assumes tokens is a flat list. This will only work if tokens is a flat list.
        return self.condenseRaces(self.condenseDeadline(self.condenseParallels(self.condenseSequentials(tokens))))

    def condenseSequentials(self, tokens):
        i = 1
        while i < len(tokens) - 1:
            if tokens[i] == Token.PLUS:
                break
            i += 1
        else:
            return tokens
        prev = tokens[i - 1]
        next = tokens[i + 1]
        return self.condenseSequentials(tokens[:i-1] + [prev.andThen(next)] + tokens[i + 2:])

    def condenseParallels(self, tokens):
        i = 1
        while i < len(tokens) - 1:
            if tokens[i] == Token.AND:
                break
            i += 1
        else:
            return tokens
        prev = tokens[i - 1]
        next = tokens[i + 1]
        return self.condenseParallels(tokens[:i-1] + [prev.alongWith(next)] + tokens[i + 2:])

    def condenseDeadline(self, tokens):
        i = 1
        while i < len(tokens) - 1:
            if tokens[i] == Token.DEADLINE:
                break
            i += 1
        else:
            return tokens
        prev = tokens[i - 1]
        next = tokens[i + 1]
        return self.condenseDeadline(tokens[:i-1] + [prev.deadlineFor(next)] + tokens[i + 2:])

    def condenseRaces(self, tokens):
        i = 1
        while i < len(tokens) - 1:
            if tokens[i] == Token.RACE:
                break
            i += 1
        else:
            return tokens
        prev = tokens[i - 1]
        next = tokens[i + 1]
        return self.condenseRaces(tokens[:i-1] + [prev.raceWith(next)] + tokens[i + 2:])

    def parse(self):
        return self.find_groups(self.tokens)[0]

def pp_json(json):
    return f'{{"version":"2025.0","command":{json},"resetOdom":false,"folder":null,"choreoAuto":false}}'

lines = []
for line in text:
    if not line:
        continue
    scanner = Scanner(line)
    scanner.scan()
    # now to group, but how?
    parser = Parser(scanner.tokens)
    lines.append(parser.parse())
cmd = Sequential(lines)

print(pp_json(cmd.json()))
