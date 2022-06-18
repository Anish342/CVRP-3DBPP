from ortools.linear_solver import pywraplp
import cvrp


def main():
    cvrp.main()
    route_map = cvrp.route_map_fun()
    data = cvrp.create_data_model()

    # Create the mip solver with the SCIP backend.
    solver = pywraplp.Solver.CreateSolver('SCIP')

    N = cvrp.m #number of cartons
    m = cvrp.v  #number of containers
    M = 100000 #arbitary large number

    #Parameters indicating the length, width, height and weight of cartons respectively.
    p=data['length_of_boxes']
    q=data['width_of_boxes']
    r=data['height_of_boxes']
    weight_of_cartons=data['weights']

    #Parameters indicating the length, width, height and maximum capacity of containers respectively.
    L=data['vehicle_lengths']
    W=data['vehicle_widths']
    H=data['vehicle_heights']
    capacity=data['vehicle_capacity']

    # Variables

    # s[i, j] = 1 if carton i is packed in container j.
    s = {}
    for i in range(N):
        for j in range(m):
            s[(i, j)] = solver.IntVar(0, 1, 's_%i_%i' % (i, j))
    
   
    #A dict for storing the carton in the exact container requried.       
    list_of_items_in_container={}
    for j in range(m):
        route=route_map[j][1:-1]          
        list_of_items_in_container[j]=[]      
        for i in route:
            list_of_items_in_container[j].append(data['boxes_at_location'][i])
        
        list_of_items_in_container[j] = [ item for elem in list_of_items_in_container[j] for item in elem]  
    
    # n[j] = 1 if container j is used.
    n = {}
    for j in range(m):
        n[j] = solver.IntVar(0,1,'n_%i'%j)
    
    #Position of the front left bottom corner of carton.
    x={}
    y={}
    z={}
    infinity = solver.infinity()
    for i in range(N):
        x[i] = solver.IntVar(0, infinity, 'x_%i'% i)  
        y[i] = solver.IntVar(0, infinity, 'y_%i'% i)
        z[i] = solver.IntVar(0, infinity, 'z_%i'% i)        

    #Binary variables indicating whether the length of carton i is parallel to the X-, Y-, or Z-axis.
    l={}
    w={}
    h={}
    for j in range(N):
        l[('x',j)] = solver.IntVar(0, 1, 'l_x_%i' % (j))
        #l[('y',j)] = solver.IntVar(0, 1, 'l_y_%i' % (j))
        l[('z',j)] = solver.IntVar(0, 1, 'l_z_%i' % (j))
        #w[('x',j)] = solver.IntVar(0, 1, 'w_x_%i' % (j))
        w[('y',j)] = solver.IntVar(0, 1, 'w_y_%i' % (j))
        #w[('z',j)] = solver.IntVar(0, 1, 'w_z_%i' % (j))
        #h[('x',j)] = solver.IntVar(0, 1, 'h_x_%i' % (j))
        #h[('y',j)] = solver.IntVar(0, 1, 'h_y_%i' % (j))
        h[('z',j)] = solver.IntVar(0, 1, 'h_z_%i' % (j))

    #Binary variables that are defined to indicate the placement of cartons relative to each other.
    ''' The a[(i,k)] is equal to 1 if box i is on the left side of carton k. Similarly, the variables b[(i,k)], c[(i,k)], d[(i,k)] , e[(i,k)] , and f[(i,k)]
    represent whether carton i is on the right of, behind, in front of, below, or above carton k, respectively. '''
    a={} 
    b={}
    c={}
    d={}
    e={}
    f={}
    for i in range(N):
        for k in range(i+1,N):
            a[(i,k)] = solver.IntVar(0, 1, 'a_%i_%i' % (i,k))
            b[(i,k)] = solver.IntVar(0, 1, 'b_%i_%i' % (i,k))
            c[(i,k)] = solver.IntVar(0, 1, 'c_%i_%i' % (i,k))
            d[(i,k)] = solver.IntVar(0, 1, 'd_%i_%i' % (i,k))
            e[(i,k)] = solver.IntVar(0, 1, 'e_%i_%i' % (i,k))
            f[(i,k)] = solver.IntVar(0, 1, 'f_%i_%i' % (i,k))

    #print('\n\nNumber of variables =', solver.NumVariables())

    # Constraints using the above variables
    for i in range(N):
        for k in range(N):
            if i<k:
                solver.Add(x[i] + p[i]*l[('x',i)] + q[i]*(l[('z',i)] - w[('y',i)] + h[('z',i)]) + r[i]*(1 - l[('x',i)] - l[('z',i)] + w[('y',i)] - h[('z',i)]) <= x[k] + (1-a[(i,k)])*M)
                solver.Add(x[k] + p[k]*l[('x',k)] + q[k]*(l[('z',k)] - w[('y',k)] + h[('z',k)]) + r[k]*(1 - l[('x',k)] - l[('z',k)] + w[('y',k)] - h[('z',k)]) <= x[i] + (1-b[(i,k)])*M)
                solver.Add(y[i] + p[i]*(1 - l[('x',i)] - l[('z',i)]) + q[i]*w[('y',i)] + r[i]*(l[('x',i)] + l[('z',i)] - w[('y',i)]) <= y[k] + (1-c[(i,k)])*M)
                solver.Add(y[k] + p[k]*(1 - l[('x',k)] - l[('z',k)]) + q[k]*w[('y',k)] + r[k]*(l[('x',k)] + l[('z',k)] - w[('y',k)]) <= y[i] + (1-d[(i,k)])*M)
                solver.Add(z[i] + p[i]*l[('z',i)] + q[i]*(1- l[('z',i)] - h[('z',i)]) + r[i]*h[('z',i)] <= z[k] + (1-e[(i,k)])*M)
                solver.Add(z[k] + p[k]*l[('z',k)] + q[k]*(1- l[('z',k)] - h[('z',k)]) + r[k]*h[('z',k)] <= z[i] + (1-f[(i,k)])*M)
                

    for i in range(N):
        for k in range(i+1,N):
            for j in range(m):
                solver.Add(a[(i,k)] + b[(i,k)] + c[(i,k)] + d[(i,k)] + e[(i,k)] + f[(i,k)] >= s[(i,j)] + s[(k,j)] - 1)

    for i in range(N):
        solver.Add(sum([s[(i,j)] for j in range(m)]) == 1)

    for i in range(N):
        for j in range(m):
            solver.Add(x[i] + p[i]*l[('x',i)] + q[i]*(l[('z',i)] - w[('y',i)] + h[('z',i)]) + r[i]*(1 - l[('x',i)] - l[('z',i)] + w[('y',i)] - h[('z',i)]) <= L[j] + (1 - s[(i,j)])*M)
            solver.Add(y[i] + p[i]*(1 - l[('x',i)] - l[('z',i)]) + q[i]*w[('y',i)] + r[i]*(l[('x',i)] + l[('z',i)] - w[('y',i)]) <= W[j] + (1 - s[(i,j)])*M)
            solver.Add(z[i] + p[i]*l[('z',i)] + q[i]*(1 - l[('z',i)] - h[('z',i)]) + r[i]*h[('z',i)] <= H[j] + (1 - s[(i,j)])*M)
    
    
    for j in range(m):
        solver.Add(sum(s[(i, j)] * weight_of_cartons[i] for i in range(N)) <= n[j] * capacity[j])    
    
    for j in range(m):
        for i in list_of_items_in_container[j]:
            solver.Add(s[(i,j)] == 1)
    
    for j in range(m):
        order = list_of_items_in_container[j]
        for i in range(len(order)):
            for j in range(len(order)):
                if j>i:
                    solver.Add(y[order[i]] >= y[order[j]] + 1)


    #print('Number of constraints =', solver.NumConstraints())

    #Objective Function
    solver.Minimize(sum([L[j]*W[j]*H[j]*n[j] for j in range(m)])- sum([p[i]*q[i]*r[i] for i in range(N)]))

    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        print('Packing as follows:')
        #print('Objective value =', solver.Objective().Value()) 
        for j in range(m):
            print('\nVehicle',j)
            for i in range(N):
                if(s[(i,j)].solution_value()==1):
                    if l[('x',i)].solution_value() == 1:
                        print("Box",i,'is packed and the x,y,z values w.r.t container','({},{},{})'.format(x[i].solution_value(),y[i].solution_value(),z[i].solution_value()),'and Length of the box parallel to x-axis.')
                    elif l[('z',i)].solution_value() == 1:
                        print("Box",i,'is packed and the x,y,z values w.r.t container','({},{},{})'.format(x[i].solution_value(),y[i].solution_value(),z[i].solution_value()),'and Length of the box parellel to z-axis.')
                    else:
                        print("Box",i,'is packed and the x,y,z values w.r.t container','({},{},{})'.format(x[i].solution_value(),y[i].solution_value(),z[i].solution_value()),'and Length of the box parallel to y-axis.')
    else:
        print('The problem does not have an optimal solution.')


    print('\nAdvanced usage:')
    print('Problem solved in %f milliseconds' % solver.wall_time())
    print('Problem solved in %d iterations' % solver.iterations())
    print('Problem solved in %d branch-and-bound nodes' % solver.nodes())


if __name__ == '__main__':
    main()
    