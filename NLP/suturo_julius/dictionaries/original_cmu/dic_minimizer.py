import random

if __name__ == "__main__":
    source = "CMU_Sphinx_dic_edit.txt"
    target = "CMU_Sphinx_dic_mini.txt"

    s = open(source, mode="r")
    content = s.readlines()
    s.close()
    t = open(target, mode="w")
    t.flush()
    log = open("log.txt", mode="w")
    log.flush()
    for line in content:
        if random.randint(0, 7) == 0:
            t.write(line)

