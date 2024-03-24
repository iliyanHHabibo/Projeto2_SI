from searchPlus import *

line1 = "= = = = = = =\n"
line2 = "= x . . . . =\n"
line3 = "= . . . = . =\n"
line4 = "= . . . = . =\n"
line5 = "= = = . = . =\n"
line6 = "= ^ . . . . =\n"
line7 = "= = = = = = =\n"
grelha = line1 + line2 + line3 + line4 + line5 + line6 + line7

class Labirinto(Problem):

    def process_txt(self, grid):
        data = {'walls': set()}
        lines = grid.split('\n')
        x = 0
        for row in lines:
            y = 0
            for col in row:
                if col == '=':
                    data['walls'].add((x,y))
                elif col == 'x':
                    data['exit'] = (x,y)
                elif col == '^':
                    data['car'] = (x,y)
                    data['direction'] = 'N'
                elif col == '>':
                    data['car'] = (x,y)
                    data['direction'] = 'E'
                elif col == '<':
                    data['car'] = (x,y)
                    data['direction'] = 'O'
                elif col == 'v':
                    data['car'] = (x,y)
                    data['direction'] = 'S'
                if col != " ":
                    y += 1
            x += 1
        data['dim'] = (len(lines)-1,(len(lines[0])+1)//2)
        return data

    
    directions = {"N":(-1, 0), "E":(0, +1), "S":(+1, 0), "O":(0, -1)}  # ortogonals

    
    def __init__(self, LabInicial=grelha, vmax=3):
        initialStatus = self.process_txt(LabInicial) # process txt and convert to a dictionary
        
        self.initial = (initialStatus['car'], initialStatus['direction'], 0)  # tuple with car position, direction and velocity
        # self.direction = initialStatus['direction'] # car direction (N, S, E, W)
        self.goal = initialStatus['exit'] # goal position
        self.obstacles = initialStatus['walls'] # walls positions
        self.dim = initialStatus['dim'] # maze dimension (do not need to be squared)
        self.vmax = vmax

        
    def actions (self, state):
        """L = rotate the car 90º left; R = rotate the car 90º right; 
         M = move forward and add 1 unit to the velocity; B = break and decrease 1 unit to the velocity"""
        x, y = state[0] # car position
        d = state[1] # direction
        v = state[2] # velocity
        action_list = []
        
        # if the car is facing an obstacle, rotate the car 90º 
        if v == 0:
            action_list.extend(['E','D'])
            
        # if the car is facing no obstacles and it can move the number of spaces given by velocity+1
        pos = [(x+(a+1)*self.directions[d][0], y+(a+1)*self.directions[d][1]) for a in range(0,v+1)]
        if all([a not in self.obstacles for a in pos]) \
        and 0 < x+(v+1)*self.directions[d][0] < self.dim[0] \
        and 0 < y+(v+1)*self.directions[d][1] < self.dim[1] :
            action_list.append('A')
            
        # if velocity is higher than 0, break is a possible action if it can move the number of spaces given by velocity-1
        pos = [(x+(a-1)*self.directions[d][0], y+(a-1)*self.directions[d][1]) for a in range(0,v+1)]
        if v > 0 and all([a not in self.obstacles for a in pos]): 
            action_list.append('T')
            
        return sorted(action_list)

    
    def result (self, state, action):
        x, y = state[0]
        d = state[1]
        v = state[2]
        if action == 'T' and v > 0:
            v -= 1
            pos = (x+v*self.directions[d][0], y+v*self.directions[d][1])
            new_dir = d
        if action == 'E':
            new_dir = list(self.directions.keys())[list(self.directions.keys()).index(d)-1]
            pos = (x, y)
        if action == 'D':
            new_dir = list(self.directions.keys())[list(self.directions.keys()).index(d)+1 if d!='O' else 0]
            pos = (x, y)
        if action == 'A':
            if v < self.vmax:
                v += 1
            pos = (x+v*self.directions[d][0], y+v*self.directions[d][1])
            new_dir = d
        return (pos,new_dir,v)

    
    def goal_test (self, state):
        return state[0] == self.goal and state[2] == 0

    
    def display (self, state):
        """Devolve a grelha em modo txt"""
        output = ""
        for i in range(self.dim[0]):
            for j in range(self.dim[1]):
                if state[0] == (i,j) and state[1] == 'N':
                    ch = "^"
                elif state[0] == (i,j) and state[1] == 'S':
                    ch = "v"
                elif state[0] == (i,j) and state[1] == 'E':
                    ch = ">"
                elif state[0] == (i,j) and state[1] == 'O':
                    ch = "<"
                elif self.goal == (i,j):
                    ch = "x"
                elif (i,j) in self.obstacles:
                    ch = "="
                else:
                    ch = "."
                output += ch + " "
            output += "\n"
        return output

    
    def executa(self, state, actions_list, verbose=False):
        """Executa uma sequência de acções a partir do estado devolvendo o triplo formado pelo estado, 
        pelo custo acumulado e pelo booleano que indica se o objectivo foi ou não atingido. Se o objectivo 
        for atingido antes da sequência ser atingida, devolve-se o estado e o custo corrente.
        Há o modo verboso e o não verboso, por defeito."""
        cost = 0
        for a in actions_list:
            seg = self.result(state,a)
            cost = self.path_cost(cost,state,a,seg)
            state = seg
            obj = self.goal_test(state)
            if verbose:
                print('Ação:', a)
                print(self.display(state),end='')
                print('Custo Total:',cost)
                print('Atingido o objectivo?', obj)
                print()
            if obj:
                break
        return (state, cost, obj)

    def comparar_orientacao(self,state):
        """
        given that the car is in the same line/column as the goal but not in the goal itself, we need to check if the car is facing the goal.
        if the goal is north from the car and the car is facing north, then the minRotacoes heuristic value is 0.
        if the goal is north from the car and the car is facing south, then the  minRotacoes heuristic value is 2.
        if the goal is east from the car and the car is facing east or west, then the minRotacoes heuristic value is 1.
        return heuristic value based on that.
        """
        x,y = state[0][0], state[0][1]
        gx, gy = self.goal[0], self.goal[1]

        #car is in the same line as the goal (car can be east or west of the goal)
        if x == gx:
            #car is east of the goal
            if y < gy:
                if state[1] == 'O': #car is facing west (the opposite direction of the goal. requires 2 rotations to face goal)
                    return 2
                elif state[1] == 'N' or state[1] == 'S': #car is facing north or south (car only requires 1 rotation to face goal)
                    return 1
                else:
                    return 0
            #car is west of the goal
            else:
                if state[1] == 'E':
                    return 2
                elif state[1] == 'N' or state[1] == 'S':
                    return 1
                else:
                    return 0
                
        #car is in the same column as the goal (car can be north or south of the goal)
        elif y == gy:
            #car is north of the goal
            if x < gx:
                if state[1] == 'N': #car is facing north (the opposite direction of the goal. requires 2 rotations to face goal)
                    return 2
                elif state[1] == 'E' or state[1] == 'O': #car is facing east or west (car only requires 1 rotation to face goal)
                    return 1
                else:
                    return 0
                
            #car is south of the goal
            else:
                if state[1] == 'S':
                    return 2
                elif state[1] == 'E' or state[1] == 'O':
                    return 1
                else:
                    return 0




    def minRotacoes(self,node):
        state = node.state
        coordinates = state[0]
        heuristic_value = 0

        #if car hits goal (gets to the destination and has speed 0) then heuristic value is 0
        if self.goal_test(state):
            pass
        
        #if car is in the goal position but has speed different from 0, then heuristic value is 2
        elif state[0] == self.goal and state[2] != 0:
            heuristic_value += 2
            
        #case where the car is in the same line/column as the goal.
        #we need to check if the car is facing the goal
        elif coordinates[0] == self.goal[0] or coordinates[1] == self.goal[1]:
            heuristic_value += self.comparar_orientacao(state)
            
        #else- todas as outras condições nao abordadas nao precisam de ser detalhadas e podemos so adicionar 2 ao heuristic value
        else:
            heuristic_value += 2

        return heuristic_value

    

    
    def minAeTs(self,node):
        pass
    



#codigo de minRotacoes so nao passa ao quinto exemplo de teste