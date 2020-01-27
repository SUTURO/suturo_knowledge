if __name__ == "__main__":
    source = "CMU_Sphinx_dic_original.txt"
    target = "CMU_Sphinx_dic_edit.txt"

    s = open(source, mode="r")
    content = s.readlines()
    s.close()
    t = open(target, mode="w")
    t.flush()
    log = open("log.txt", mode="w")
    log.flush()
    for line in content:
        if not (line.__contains__("(") or line.__contains__("!") or line.__contains__(")") or line.__contains__("ey z")): # TODO add more
            lineSplit = line.split("\t")
            if len(lineSplit[0]) < 8:
                lineSplit[1] = lineSplit[1].lower()
                t.write(lineSplit[0] + "\t" + lineSplit[1])
        else:
            log.write(line)  # I write cut lines to this file
