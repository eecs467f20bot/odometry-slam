a, b = b, a


a, *b, c = (1,2,3,4)
# a = 1
# b = (2,3)
# c = 4

d = defaultdict(lambda: -1)
# d["a"] -> -1

def allTheArgs(*argv):
    for arg in argv:
        print("I have you now, ", arg)


allTheArgs("Peter Pan", "Wendy", "Tinker Bell")