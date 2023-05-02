import math

# Define the distance function
def dist(u, v):
    return math.sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)

# Define the k-robot assignment algorithm
def k_robot_assignment(W, L):
    n = len(W)
    k = len(W[0])
    T = [[] for i in range(k)]
    T_dep = [None for i in range(k)]
    x_dep = [None for i in range(k)]
    for i in range(n):
        w = W[i]
        min_cost = float('inf')
        min_j = None
        for j in range(k):
            cost = dist(w, x_dep[j]) + L
            if cost < min_cost:
                min_cost = cost
                min_j = j
        T[min_j].append(i)
        if T_dep[min_j] is None or w[1] > W[T_dep[min_j]][0][1]:
            T_dep[min_j] = len(T[min_j])-1
            x_dep[min_j] = w
    for j in range(k):
        if T_dep[j] is None:
            return False
        T_dep[j] = T[j][T_dep[j]]
    return T, T_dep, x_dep

# Define the single-robot schedule algorithm
def single_robot_schedule(T, T_dep):
    n = len(T)
    L = 0
    for i in range(n):
        w = T[i][0][1]
        L = max(L, dist(T[i][T_dep[i]], T[i][0]) + w)
    return L

# Define the main function
def main():
    # Define the weights of the nodes
    W = [(1, 3), (2, 2), (3, 1), (4, 2), (5, 3)]
    # Define the number of robots
    k = 2
    # Compute the smallest and largest weights
    w_min = min(W, key=lambda x: x[1])[1]
    w_max = max(W, key=lambda x: x[1])[1]
    # Compute the value of m
    m = math.log2(w_max/w_min)
    # Round the weights to the nearest dyadic value
    W = [(w[0], 2**math.ceil(math.log2(w[1]))) for w in W]
    # Run the k-robot assignment algorithm
    L = 6 # Guess an upper bound for the optimal maximum weighted latency
    T = None
    while T is None:
        T = k_robot_assignment(W, L)
        if T is None:
            L *= 2
    # Compute the maximum weighted latency of the schedule
    max_latency = 0
    for j in range(k):
        T_j = [W[i] for i in T[j]]
        T_dep_j = T.index(T_dep[j])
        max_latency = max(max_latency, single_robot_schedule(T_j, T_dep_j))
    # Print the results
    print('Weights:', W)
    print('Number of robots:', k)
    print('Maximum weighted latency:', max_latency)

if __name__ == '__main__':
    main()
