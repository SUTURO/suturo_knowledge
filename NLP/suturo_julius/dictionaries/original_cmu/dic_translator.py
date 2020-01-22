if __name__ == "__main__":
    source = "CMU_Sphinx_dic.txt"
    target = "CMU_Sphinx_dic_edit.txt"

    s = open(source, mode="r")
    content = s.readlines()
    s.close()
    t = open(target, mode="w")
    t.flush()
    log = open("log.txt", mode="w")
    log.flush()
    for line in content:
        if line.__contains__("(2)"): # TODO regex
            lineSplit = line.split("\t")
            lineSplit[1] = lineSplit[1].lower()
            lineSplit[0] = lineSplit[0][0:-3]
            t.write(lineSplit[0] + "\t" + lineSplit[1])
        if not (line.__contains__("(") or line.__contains__("!") or line.__contains__(")")): # TODO add more
            lineSplit = line.split("\t")
            lineSplit[1] = lineSplit[1].lower()
            t.write(lineSplit[0] + "\t" + lineSplit[1])
        else:
            log.write(line) # I write cut lines to this file
