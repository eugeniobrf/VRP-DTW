import matplotlib.pyplot as plt
import pandas as pd
import sys

def addArrow(ax1, xInicio, yInicio, xFim, yFim, cor, legenda, lineStyle='-'):
    if legenda == "":
        ax1.arrow(xInicio, yInicio,(xFim-xInicio) , (yFim-yInicio), width = 0.001, head_width = 0.5, head_length = 0.5, length_includes_head=True, color=cor, linestyle=lineStyle)
    else:
        ax1.arrow(xInicio, yInicio,(xFim-xInicio) , (yFim-yInicio), width = 0.001, head_width = 0.5, head_length = 0.5, length_includes_head=True, color=cor, label=legenda, linestyle=lineStyle)

if __name__ == '__main__':
    instance = sys.argv[1]
    solution = sys.argv[2]
    
    f = open(instance, "r")
    numLines = int(f.readline())
    numVertex = numLines
    capacity = f.readline()
    f.readline()
    x = []
    y = []
    initTimeWindow = []
    endTimeWindow = []
    drone = []
    request = []
    for i in range(numLines):
        line = f.readline().split(",")
        x.append(float(line[2]))
        y.append(float(line[3]))
        request.append(line[6])
        drone.append(int(line[9]))
        initTimeWindow.append(float(line[4]))
        endTimeWindow.append(float(line[5]))
    f.close()
    
    fig = plt.figure()
    ax1 = fig.add_subplot()
    ax = plt.gca()

    firstDrone = True
    firstTruck = True
    
    rangeX = max(x)-min(x)
    rangeY = max(y)-min(y)

    if(rangeX > rangeY):
        space = rangeX - rangeY
        plt.xlim([min(x)-5, max(x)+5])
        plt.ylim([min(y)-space/2 - 5, max(y)+space/2 + 5])
    else:
        space = rangeY - rangeX
        plt.ylim([min(y)-5, max(y)+5])
        plt.xlim([min(x)-space/2 - 5, max(x)+space/2 + 5])
    
    plt.plot(x[0], y[0], 'gs', label='Deposito')
    for i in range(len(x)-1):
        if (drone[i+1]==1):
            if(firstDrone):
                plt.plot(x[i+1],y[i+1],'o', color='aqua', label='Clientes passiveis de drone')
                firstDrone = False
            else:
                plt.plot(x[i+1],y[i+1],'o', color='aqua')
        else:
            if(firstTruck):
                plt.plot(x[i+1],y[i+1],'o', color='lime', label='Clientes não passiveis de drone')
                firstTruck = False
            else:
                plt.plot(x[i+1],y[i+1],'o', color='lime')
    for i in range(len(x)-1):
        ax.annotate("["+str(int(initTimeWindow[i+1]))+","+str(int(endTimeWindow[i+1]))+"]", (x[i+1],y[i+1]+0.1), size=6, horizontalalignment='center', verticalalignment='bottom')
        ax.annotate(request[i+1], (x[i+1],y[i+1]-0.1), size=6, horizontalalignment='center', verticalalignment='top')
    f = open(solution, "r");
    numLines = int(f.readline())
    cost = f.readline()
    for i in range(numLines):
        line = f.readline()[:-1].split("-")
        for j in range(len(line)-1):
            inicio = int(line[j])
            fim = int(line[j+1])
            if fim==numVertex:
                fim=0
            if(j==0 and i==0):
                addArrow(ax1,x[inicio],y[inicio],x[fim],y[fim],'coral','Truck')
            else:
                addArrow(ax1,x[inicio],y[inicio],x[fim],y[fim],'coral','')
        line = f.readline()[:-1].split("-")
        for j in range(1,len(line),3):
            inicio = int(line[j-1])
            atendido = int(line[j])
            fim = int(line[j+1])
            if fim==numVertex:
                fim=0
            if inicio==numVertex:
                inicio=0
            if(j==1 and i==0):
                addArrow(ax1,x[inicio],y[inicio],x[atendido],y[atendido],'red','Drone', ':')
                addArrow(ax1,x[atendido],y[atendido],x[fim],y[fim],'red','', ':')
            else:
                addArrow(ax1,x[inicio],y[inicio],x[atendido],y[atendido],'red','',':')
                addArrow(ax1,x[atendido],y[atendido],x[fim],y[fim],'red','', ':')
    plt.legend()
    plt.title("Capacidade caminhão: " + str(capacity) + " Custo: " + str(cost))
    plt.show()
    