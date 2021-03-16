#!/usr/bin/python3

print('### Script:', __file__)

import math
import sys
import time

import sim

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Robot handle
    _,rbh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                     sim.simx_opmode_blocking)

    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     sim.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = sim.simxGetObjectHandle(clientID, str % (i+1),
                                       sim.simx_opmode_blocking)
        sonar[i] = h
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    return [lmh, rmh], sonar, rbh

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

count = 0
state = 0
# 0: explorando
# 1: marcha atras
# 2: meta
i_mov = 0
lista_mov = []
# 0: derecha
# 1: recto
# 2: izquierda
# 3: derecha (marcha atras)
# 4: recto (marcha atras)
# 5: izquierda (marcha atras)
i_deci = 0
lista_deci = []
# [estado, girar derecha, ir recto, girar izquierda]

def get_mov(sonar):
    global state, count, i_deci, lista_deci
    # comprueba si ha llegado a la meta
    suma = 0
    for i in range (16):
        suma = suma + sonar[i]*sonar[i]
    if suma > 10:
        count = count + 1
        if count == 30:
            return -1, -1, -1
    else:
        count = 0
    dcha, recto, izda = 0, 0, 0
    if state == 0:          # devuelve los movimientos marcha adelante
        if sonar[6] > 0.8 or sonar[7] > 0.8:
            if sonar[6] < 0.3 or sonar[7] < 0.2:
                dcha = 2   # cerca dcha
            elif sonar[0] < 0.2 or sonar[1] < 0.3:
                dcha = 3   # cerca izda
            else:
                dcha = 1
        if sonar[3] > 0.9 or sonar[4] > 0.9:
            if sonar[6] < 0.3 or sonar[7] < 0.2:
                recto = 2   # cerca dcha
            elif sonar[0] < 0.2 or sonar[1] < 0.3:
                recto = 3   # cerca izda
            else:
                recto = 1
        if sonar[0] > 0.8 or sonar[1] > 0.8:
            if sonar[6] < 0.3 or sonar[7] < 0.2:
                izda = 2   # cerca dcha
            elif sonar[0] < 0.2 or sonar[1] < 0.3:
                izda = 3   # cerca izda
            else:
                izda = 1
    else:                   # devuelve los movimientos marcha atras
        if sonar[14] > 0.8 or sonar[15] > 0.8:
            if sonar[14] < 0.3 or sonar[15] < 0.2:
                dcha = 2   # cerca dcha
            elif sonar[8] < 0.2 or sonar[9] < 0.3:
                dcha = 3   # cerca izda
            else:
                dcha = 1
        if sonar[11] > 0.9 or sonar[12] > 0.9:
            if sonar[14] < 0.3 or sonar[15] < 0.2:
                recto = 2   # cerca dcha
            elif sonar[8] < 0.2 or sonar[9] < 0.3:
                recto = 3   # cerca izda
            else:
                recto = 1
        if sonar[8] > 0.8 or sonar[9] > 0.8:
            if sonar[14] < 0.3 or sonar[15] < 0.2:
                izda = 2   # cerca dcha
            elif sonar[8] < 0.2 or sonar[9] < 0.3:
                izda = 3   # cerca izda
            else:
                izda = 1
    # guarda el nodo de decisiones
    if dcha == 0:
        dcha_aux = 0
    else:
        dcha_aux = 1
    if recto == 0:
        recto_aux = 0
    else:
        recto_aux = 1
    if izda == 0:
        izda_aux = 0
    else:
        izda_aux = 1
    if not lista_deci:
        lista_deci.append([state, dcha_aux, recto_aux, izda_aux])
    else:
        pre_state = lista_deci[i_deci][0]
        pre_dcha = lista_deci[i_deci][1]
        pre_recto = lista_deci[i_deci][2]
        pre_izda = lista_deci[i_deci][3]
        if pre_state == state:
            if pre_dcha == dcha_aux:
                if pre_recto == recto_aux:
                    if pre_izda == izda_aux:
                        return dcha, recto, izda
        lista_deci.append([state, dcha_aux, recto_aux, izda_aux])
        i_deci = i_deci + 1
    return dcha, recto, izda


def escoge_mov(dcha, recto, izda):
    global state, lista_mov, i_mov
    lspeed, rspeed = 0, 0
    # escoge el tipo de movimiento segun los datos
    if dcha > 0:
        giro = 0
    elif recto > 0:
        giro = 1
    else:
        giro = 2
    if state == 1:
        giro = giro + 3
    # velocidades de los motores segun los sensores
    if giro == 0:    # derecha normal
        if dcha == 2:
            lspeed, rspeed = 2.0, 1.7
        elif dcha == 3:
            lspeed, rspeed = 2.0, 0.5
        else:
            lspeed, rspeed = 2.0, 1.0
    elif giro == 1:    # recto normal
        if recto == 1:
            lspeed, rspeed = 2.0, 2.0
        elif recto == 2:
            lspeed, rspeed = 1.2, 2.0
        else:
            lspeed, rspeed = 2.0, 1.2
    elif giro == 2:    # izquierda normal
        if izda == 2:
            lspeed, rspeed = 0.5, 2.0
        if izda == 3:
            lspeed, rspeed = 1.7, 2.0
        else:
            lspeed, rspeed = 1.0, 2.0
    elif giro == 3:    # derecha atras
        if dcha == 2:
            lspeed, rspeed = -1.7, -2.0
        elif dcha == 3:
            lspeed, rspeed = -0.5, -2.0
        else:
            lspeed, rspeed = -1.0, -2.0
    elif giro == 4:    # recto atras
        if recto == 1:
            lspeed, rspeed = -2.0, -2.0
        elif recto == 2:
            lspeed, rspeed = -2.0, -1.2
        else:
            lspeed, rspeed = -1.2, -2.0
    elif giro == 5:    # izquierda atras
        if izda == 2:
            lspeed, rspeed = -2.0, -0.5
        elif izda == 3:
            lspeed, rspeed = -2.0, -1.7
        else:
            lspeed, rspeed = -2.0, -1.0
    if not lista_mov:
        lista_mov.append(giro)
    elif not lista_mov[i_mov] == giro:
        lista_mov.append(giro)
        i_mov = i_mov + 1
    return lspeed, rspeed


def explore(sonar):
    global state
    dcha, recto, izda = get_mov(sonar)
    if dcha == -1:
        state = 2
        return goal()
    if dcha == 0 and izda == 0 and recto == 0:
        if state == 0:
            state = 1
            dcha, recto, izda = get_mov(sonar)
        else:
            state = 0
            dcha, recto, izda = get_mov(sonar)
    return escoge_mov(dcha, recto, izda)


def goal():
    global lista_mov, lista_deci, count
    if count == 30:
        print ('Lista de movimientos realizados: ')
        print (lista_mov)
        print ('Lista de nodos de decisiones: ')
        print (lista_deci)
        count = 0
    return 2.0, -2.0


def tratar(sonar):
    global state
    if state == 0 or state == 1:
        lspeed, rspeed = explore(sonar)
    else:
        lspeed, rspeed = goal()
    return lspeed, rspeed

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            # print '### s', sonar

            # Planning
            lspeed, rspeed = tratar(sonar)

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
