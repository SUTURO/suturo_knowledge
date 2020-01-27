import sys

if __name__ == "__main__":
    source = sys.argv[1]

    s = open(source + ".txt", mode="r")
    content = s.readlines()
    s.close()
    head = open(source + "_1.txt", mode="w")
    head.flush()
    tail = open(source + "_2.txt", mode="w")
    tail.flush()
    contentHead = content[:len(content)/2]
    contentTail = content[len(content) / 2:]
    for line in contentHead:
        head.write(line)
    head.close()
    for line in contentTail:
        tail.write(line)
    tail.close()


