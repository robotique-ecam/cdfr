from random import *


class Table:
    def __init__(self, zonenonR):
        self.ecueilejaune = ['v', 'r', 'v', 'r', 'v']
        self.ecueileneutre = zonenonR
        self.chenalV = 6*[None]
        self.chenalR = 6*[None]
        self.port = 6*[None]

    def score(self):
        nbp = 0
        nbv = 0
        nbr = 0
        for i in range(0, 6):
            if self.chenalV[i] == 'v':
                nbv = nbv+1
                nbp = nbp+2
            if self.chenalR[i] == 'r':
                nbr = nbr+1
                nbp = nbp+2
            if self.port[i] == 'r' or self.port[i] == 'v':
                nbp = nbp+1
            if self.chenalR[i] == 'v' or self.chenalV[i] == 'r':
                nbp = nbp+1
        if nbv >= nbr:
            nbp = nbp+nbr
        else:
            nbp = nbp+nbv
        return(nbp)


class Robot:
    def __init__(self, Table):
        self.gobelets = 5*[None]
        self.Table = Table
        self.actions = 0
        self.actupose = "port"
        self.histo = ["port"]

    def rotation(self):
        self.gobelets[::-1]
        self.histo = self.histo+["rotation"]
        self.actions = self.actions+1

    def bouger(self, newpose):
        self.histo = self.histo+[newpose]
        self.actupose = newpose
        self.actions = self.actions+3

    def prendreG(self, liste, dep):
        self.histo = self.histo+["ventouseON"]+[liste]+[dep]
        if self.actupose == "ecueileneutre":
            for i in range(0, 5):
                if liste[i] == 1 and self.gobelets[i] is None:
                    self.gobelets[i] = self.Table.ecueileneutre[i]
                    self.Table.ecueileneutre[i] = None
        elif self.actupose == "ecueilejaune":
            for i in range(0, 5):
                if liste[i] == 1 and self.gobelets[i] is None:
                    self.gobelets[i] = self.Table.ecueilejaune[i]
                    self.Table.ecueilejaune[i] = None
        elif self.actupose == "port":
            for i in range(0, 5):
                if liste[i] == 1 and self.gobelets[i] is None:
                    self.gobelets[i] = self.Table.port[i+dep]
                    self.Table.port[i+dep] = None
        elif self.actupose == "chenalR":
            for i in range(0, 5):
                if liste[i] == 1 and self.gobelets[i] is None:
                    self.gobelets[i] = self.Table.chenalR[i+dep]
                    self.Table.chenalR[i+dep] = None
        elif self.actupose == "chenalV":
            for i in range(0, 5):
                if liste[i] == 1 and self.gobelets[i] is None:
                    self.gobelets[i] = self.Table.chenalV[i+dep]
                    self.Table.chenalV[i+dep] = None

    def poserG(self, liste, dep):
        self.histo = self.histo+["ventouseOF"]+[liste]+[dep]
        if self.actupose == "port":
            for i in range(0, 5):
                if liste[i] == 1 and self.Table.port[i+dep] is None:
                    self.Table.port[i+dep] = self.gobelets[i]
                    self.gobelets[i] = None
        elif self.actupose == "chenalR":
            for i in range(0, 5):
                if liste[i] == 1 and self.Table.chenalR[i+dep] is None:
                    self.Table.chenalR[i+dep] = self.gobelets[i]
                    self.gobelets[i] = None
        elif self.actupose == "chenalV":
            for i in range(0, 5):
                if liste[i] == 1 and self.Table.chenalV[i+dep] is None:
                    self.Table.chenalV[i+dep] = self.gobelets[i]
                    self.gobelets[i] = None

    def final(self):
        fatality = [self.histo, self.actions, self.Table.score()]
        return(fatality)


def randomListe():
    list = []
    for i in range(0, 5):
        list = list+[randint(0, 1)]
    return(list)


L1 = ['v', 'r', 'r', 'v', 'r']
L2 = ['v', 'r', 'v', 'r', 'r']
L3 = ['v', 'v', 'r', 'r', 'r']
Table1 = Table(L1)
Table2 = Table(L2)
Table3 = Table(L3)


Grodebelix = Robot(Table1)

Grodebelix.poserG(randomListe(), 0)
best = Grodebelix.final()
best1 = Grodebelix

while best[2] < 24:
    Table1 = Table(L1)
    Grodebelix = Robot(Table1)
    Grodebelix.bouger("ecueileneutre")
    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
    for k in range(0, 4):
        if k == 0:
            Grodebelix.bouger("chenalR")
            Grodebelix.poserG(randomListe(), randint(0, 1))
            for z in range(0, 3):
                if z == 0:
                    Grodebelix.bouger("chenalV")
                    Grodebelix.poserG(randomListe(), randint(0, 1))
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalR")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.bouger("port")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.rotation()
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                elif z == 1:
                    Grodebelix.bouger("port")
                    Grodebelix.poserG(randomListe(), randint(0, 1))
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalR")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.bouger("chenalV")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.rotation()
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                elif z == 2:
                    Grodebelix.rotation()
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalR")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.bouger("port")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.bouger("chenalV")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
        elif k == 1:
            Grodebelix.bouger("chenalV")
            Grodebelix.poserG(randomListe(), randint(0, 1))
            for z in range(0, 3):
                if z == 0:
                    Grodebelix.bouger("chenalR")
                    Grodebelix.poserG(randomListe(), randint(0, 1))
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalR")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.rotation()
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.bouger("port")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                elif z == 1:
                    Grodebelix.bouger("port")
                    Grodebelix.poserG(randomListe(), randint(0, 1))
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalR")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.rotation()
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.bouger("chenalV")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                elif z == 2:
                    Grodebelix.rotation()
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalR")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.bouger("port")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.bouger("chenalV")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
        elif k == 2:
            Grodebelix.bouger("port")
            Grodebelix.poserG(randomListe(), randint(0, 1))
            for z in range(0, 3):
                if z == 0:
                    Grodebelix.bouger("chenalR")
                    Grodebelix.poserG(randomListe(), randint(0, 1))
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalV")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.bouger("port")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.rotation()
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                elif z == 1:
                    Grodebelix.bouger("chenalV")
                    Grodebelix.poserG(randomListe(), randint(0, 1))
                    for a in range(0, 3):
                        if a == 0:
                            Grodebelix.bouger("chenalR")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 1:
                            Grodebelix.bouger("port")
                            Grodebelix.poserG(randomListe(), randint(0, 1))
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.rotation()
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                        elif a == 2:
                            Grodebelix.rotation()
                            for b in range(0, 4):
                                if b == 0:
                                    Grodebelix.bouger("chenalR")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 1:
                                    Grodebelix.bouger("port")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 2:
                                    Grodebelix.bouger("chenalV")
                                    Grodebelix.poserG(randomListe(), randint(0, 1))
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                                elif b == 3:
                                    Grodebelix.bouger("ecueilejaune")
                                    Grodebelix.prendreG(randomListe(), 0)
                                    if randint(0, 10) == 1:
                                        Table1 = Table(L1)
                                        Grodebelix = Robot(Table1)
                                        Grodebelix.bouger("ecueileneutre")
                                        Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                elif z == 2:
                    Grodebelix.rotation()
                    if a == 0:
                        Grodebelix.bouger("chenalR")
                        Grodebelix.poserG(randomListe(), randint(0, 1))
                        for b in range(0, 4):
                            if b == 0:
                                Grodebelix.bouger("chenalV")
                                Grodebelix.poserG(randomListe(), randint(0, 1))
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 1:
                                Grodebelix.bouger("port")
                                Grodebelix.poserG(randomListe(), randint(0, 1))
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 2:
                                Grodebelix.rotation()
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 3:
                                Grodebelix.bouger("ecueilejaune")
                                Grodebelix.prendreG(randomListe(), 0)
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                    elif a == 1:
                        Grodebelix.bouger("port")
                        Grodebelix.poserG(randomListe(), randint(0, 1))
                        for b in range(0, 4):
                            if b == 0:
                                Grodebelix.bouger("chenalR")
                                Grodebelix.poserG(randomListe(), randint(0, 1))
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 1:
                                Grodebelix.bouger("chenalV")
                                Grodebelix.poserG(randomListe(), randint(0, 1))
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 2:
                                Grodebelix.rotation()
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 3:
                                Grodebelix.bouger("ecueilejaune")
                                Grodebelix.prendreG(randomListe(), 0)
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                    elif a == 2:
                        Grodebelix.bouger("chenalV")
                        Grodebelix.poserG(randomListe(), randint(0, 1))
                        for b in range(0, 4):
                            if b == 0:
                                Grodebelix.bouger("chenalR")
                                Grodebelix.poserG(randomListe(), randint(0, 1))
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 1:
                                Grodebelix.bouger("port")
                                Grodebelix.poserG(randomListe(), randint(0, 1))
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 2:
                                Grodebelix.rotation()
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
                            elif b == 3:
                                Grodebelix.bouger("ecueilejaune")
                                Grodebelix.prendreG(randomListe(), 0)
                                if randint(0, 10) == 1:
                                    Table1 = Table(L1)
                                    Grodebelix = Robot(Table1)
                                    Grodebelix.bouger("ecueileneutre")
                                    Grodebelix.prendreG([1, 1, 1, 1, 1], 0)
    if best[2] < Grodebelix.final()[2]:
        best = Grodebelix.final()
        best1 = Grodebelix
        print(best[2])


print(best1.Table.chenalR)
print(best1.Table.chenalV)
print(best1.final())
