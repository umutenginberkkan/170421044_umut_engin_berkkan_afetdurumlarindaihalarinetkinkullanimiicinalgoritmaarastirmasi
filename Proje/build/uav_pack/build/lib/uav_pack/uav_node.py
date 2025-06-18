import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
import math
import random
import os
import csv

def geo_to_local(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    x = (lat - ref_lat) * 111320.0
    y = (lon - ref_lon) * 111320.0 * math.cos(math.radians(ref_lat))
    z = alt - ref_alt
    return x, y, z

def is_connected(pts, comm_range):
    n = len(pts)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            if math.dist((pts[i].x, pts[i].y), (pts[j].x, pts[j].y)) <= comm_range:
                adj[i].append(j)
                adj[j].append(i)
    seen = {0}
    stack = [0]
    while stack:
        u = stack.pop()
        for v in adj[u]:
            if v not in seen:
                seen.add(v)
                stack.append(v)
    return len(seen) == n

class UAV:
    def __init__(self, uav_id, pos: Point, coverage_radius, comm_range):
        self.id = uav_id
        self.position = pos
        self.coverage_radius = coverage_radius
        self.comm_range = comm_range
        self.phase = "hold"
        self.formation_offset = None
        self.dispersion_target = None

    def update(self, dt, speed, thresh, target: Point):
        dx = target.x - self.position.x
        dy = target.y - self.position.y
        dz = target.z - self.position.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < thresh:
            self.position.x = target.x
            self.position.y = target.y
            self.position.z = target.z
            return
        else:
            step = speed * dt
            self.position.x += dx / dist * step
            self.position.y += dy / dist * step
            self.position.z += dz / dist * step

class UAVSimulator(Node):
    def __init__(self):
        super().__init__('uav_simulation')
        
        self.ref_lat = 41.0
        self.ref_lon = 29.0
        self.ref_alt = 0.0
        # 1) Parametreleri oku
        self.declare_parameter('num_uav',   5)
        self.declare_parameter('formation', 1)
        # 1: PSO, 2: ACO, 3: SMA
        self.declare_parameter('algorithm', 1)
        n    = self.get_parameter('num_uav').value
        ft   = self.get_parameter('formation').value
        algo = self.get_parameter('algorithm').value
        self.algorithm = algo
        self.get_logger().info(f"Parametreler: num_uav={n}, formation={ft}, algorithm={algo}")

        self.declare_parameter('area_size', 150.0)  # GUI'den gelen
        area_size_m = self.get_parameter('area_size').value

        # Metreyi dereceye çevir (yaklaşık)
        delta_lat = area_size_m / 111320.0
        delta_lon = area_size_m / (111320.0 * math.cos(math.radians(self.ref_lat)))

        min_lat = self.ref_lat
        max_lat = self.ref_lat + delta_lat
        min_lon = self.ref_lon
        max_lon = self.ref_lon + delta_lon
        min_alt = 0.0
        max_alt = 20.0

        self.scan_x_min, self.scan_y_min, _ = geo_to_local(min_lat, min_lon, min_alt, self.ref_lat, self.ref_lon, self.ref_alt)
        self.scan_x_max, self.scan_y_max, _ = geo_to_local(max_lat, max_lon, max_alt, self.ref_lat, self.ref_lon, self.ref_alt)


        self.scan_x_min, self.scan_y_min, _ = geo_to_local(
            min_lat, min_lon, min_alt,
            self.ref_lat, self.ref_lon, self.ref_alt
        )
        self.scan_x_max, self.scan_y_max, _ = geo_to_local(
            max_lat, max_lon, max_alt,
            self.ref_lat, self.ref_lon, self.ref_alt
        )

        # 3) Flight altitude
        self.declare_parameter('flight_altitude', 20.0)
        self.flight_altitude = self.get_parameter('flight_altitude').value

        # 4) Hedef küp
        self.final_target = Point()
        self.final_target.x = (self.scan_x_min + self.scan_x_max) / 2.0
        self.final_target.y = (self.scan_y_min + self.scan_y_max) / 2.0
        self.final_target.z = self.flight_altitude

        # 5) Hold center
        self.hold_center = Point()
        self.hold_center.x = self.final_target.x + 150.0
        self.hold_center.y = self.final_target.y
        self.hold_center.z = self.flight_altitude

        # 6) Global formasyon merkezi (hold_center’ın kopyası)
        self.global_form_center = Point()
        self.global_form_center.x = self.hold_center.x
        self.global_form_center.y = self.hold_center.y
        self.global_form_center.z = self.hold_center.z

        # 7) UAV parametreleri
        for p,v in [
            ('coverage_radius',20.0),
            ('comm_threshold',50.0),
            ('speed',5.0),
            ('waypoint_threshold',0.5),
        ]:
            self.declare_parameter(p, v)
        cov  = self.get_parameter('coverage_radius').value
        comm = self.get_parameter('comm_threshold').value
        spd  = self.get_parameter('speed').value
        wth  = self.get_parameter('waypoint_threshold').value
        self.speed          = spd
        self.wp_thresh      = wth
        self.comm_threshold = comm

        # 8) Hold formation pozisyonları
        holds = self.compute_formation_targets(n, self.hold_center, ft, spacing=10.0)
        self.uavs = []
        for i,pos in enumerate(holds):
            u = UAV(i, pos, cov, comm)
            off = Point()
            off.x = pos.x - self.hold_center.x
            off.y = pos.y - self.hold_center.y
            off.z = pos.z - self.hold_center.z
            u.formation_offset = off
            self.uavs.append(u)

        # 9) Simülasyon aşamaları
        self.phase = "hold"
        self.hold_duration = 5.0
        self.start_time    = self.get_clock().now().nanoseconds / 1e9

        # 10) Publisher’lar ve timer
        self.marker_pub = self.create_publisher(MarkerArray,'uav_markers',10)
        self.gps_pubs   = {
            u.id: self.create_publisher(NavSatFix, f'uav{u.id}_gps', 10)
            for u in self.uavs
        }
        self.create_timer(0.1, self.timer_callback)
        self.declare_parameter('dispersion_iters', 1)
        self.remaining_iters = self.get_parameter('dispersion_iters').value
        self.coverage_so_far = set()
        self.iteration_counter = 1  # CSV logu için
        self.disperse_initialized = False

    def compute_formation_targets(self, n, center: Point, ft, spacing):
        pts = []
        if ft == 1:
            r,c=0,0
            while c<n:
                for j in range(r+1):
                    if c>=n: break
                    p = Point()
                    p.x = center.x + r*spacing
                    p.y = center.y - (r*spacing)/2 + j*spacing
                    p.z = center.z
                    pts.append(p)
                    c+=1
                r+=1
        elif ft == 2:
            p0 = Point()
            p0.x,p0.y,p0.z = center.x, center.y, center.z
            pts.append(p0)
            li,ri = 1,1
            for i in range(1,n):
                p = Point()
                p.z = center.z
                if i%2:
                    p.x = center.x + li*spacing
                    p.y = center.y - li*spacing
                    li+=1
                else:
                    p.x = center.x + ri*spacing
                    p.y = center.y + ri*spacing
                    ri+=1
                pts.append(p)
        else:
            for i in range(n):
                p = Point()
                p.x = center.x + (i-(n-1)/2)*spacing
                p.y = center.y
                p.z = center.z
                pts.append(p)
        return pts
    def fitness(self, pts):
        
        penalty = 0.0

        # Overlap cezası
        for i, p1 in enumerate(pts):
            for j, p2 in enumerate(pts):
                if i >= j:
                    continue
                d = math.dist((p1.x, p1.y), (p2.x, p2.y))
                min_dist = 2 * self.uavs[i].coverage_radius
                if d < min_dist:
                    penalty += (min_dist - d)**2

        # Alan dışına çıkma cezası
        for p in pts:
            if not (self.scan_x_min <= p.x <= self.scan_x_max and
                    self.scan_y_min <= p.y <= self.scan_y_max):
                penalty += 1e5

        # Bağlantı yoksa büyük ceza
        if not is_connected(pts, self.comm_threshold):
            penalty += 1e6

        return penalty
    def pso_distribute(self):
        n = len(self.uavs)
        num_particles = 20
        iters = 50
        w, c1, c2 = 0.7, 1.5, 1.5
        v_max = 5.0

        particles = []
        velocities = []
        pbest = []
        pbest_costs = []

        gbest = None
        gbest_cost = float('inf')

        for _ in range(num_particles):
            while True:
                particle = []
                velocity = []

                for _ in range(n):
                    x = random.uniform(self.scan_x_min, self.scan_x_max)
                    y = random.uniform(self.scan_y_min, self.scan_y_max)
                    pt = Point(x=x, y=y, z=self.flight_altitude)
                    particle.append(pt)

                    vx = random.uniform(-v_max, v_max)
                    vy = random.uniform(-v_max, v_max)
                    velocity.append(Point(x=vx, y=vy, z=0.0))

                if is_connected(particle, self.comm_threshold):
                    break  # bağlantılı bir parçacık bulundu

            cost = self.fitness(particle)
            particles.append(particle)
            velocities.append(velocity)

            pbest.append([Point(x=p.x, y=p.y, z=p.z) for p in particle])
            pbest_costs.append(cost)

            if cost < gbest_cost:
                gbest = [Point(x=p.x, y=p.y, z=p.z) for p in particle]
                gbest_cost = cost

        for _ in range(iters):
            for i in range(num_particles):
                for j in range(n):
                    for coord in ['x', 'y']:
                        r1, r2 = random.random(), random.random()
                        curr = getattr(particles[i][j], coord)
                        vel = getattr(velocities[i][j], coord)
                        pb = getattr(pbest[i][j], coord)
                        gb = getattr(gbest[j], coord)

                        new_vel = w * vel + c1 * r1 * (pb - curr) + c2 * r2 * (gb - curr)
                        new_vel = max(min(new_vel, v_max), -v_max)
                        setattr(velocities[i][j], coord, new_vel)

                        new_pos = curr + new_vel
                        if coord == 'x':
                            new_pos = min(max(new_pos, self.scan_x_min), self.scan_x_max)
                        else:
                            new_pos = min(max(new_pos, self.scan_y_min), self.scan_y_max)
                        setattr(particles[i][j], coord, new_pos)

                    particles[i][j].z = self.flight_altitude

                # ↪️ Güncellenen parçacık bağlantılı mı kontrol et
                if not is_connected(particles[i], self.comm_threshold):
                    continue  # bağlantısızsa değerlendirme yapma

                cost = self.fitness(particles[i])
                if cost < pbest_costs[i]:
                    pbest_costs[i] = cost
                    pbest[i] = [Point(x=p.x, y=p.y, z=p.z) for p in particles[i]]

                if cost < gbest_cost:
                    gbest_cost = cost
                    gbest = [Point(x=p.x, y=p.y, z=p.z) for p in particles[i]]

        return gbest

    def aco_distribute(self):
        num_ants = 20
        num_iters = 30
        evaporation = 0.1
        alpha = 1.0
        beta = 2.0

        grid_x = [self.scan_x_min + (i + 0.5) * (self.scan_x_max - self.scan_x_min) / 10 for i in range(10)]
        grid_y = [self.scan_y_min + (j + 0.5) * (self.scan_y_max - self.scan_y_min) / 10 for j in range(10)]
        cells = [(x, y) for x in grid_x for y in grid_y]

        pheromone = {c: 1.0 for c in cells}
        best_sol, best_cost = None, float('inf')

        for _ in range(num_iters):
            for _ in range(num_ants):
                solution = []
                available = set(cells)

                for k in range(len(self.uavs)):
                    if k == 0:
                        candidates = list(available)
                    else:
                        candidates = [
                            c for c in available
                            if any(
                                math.dist(c, (p.x, p.y)) <= self.comm_threshold
                                for p in solution
                            )
                        ]
                        if not candidates:
                            candidates = list(available)

                    weights = []
                    for cell in candidates:
                        tau = pheromone[cell] ** alpha
                        avg = sum(math.dist(cell, (p.x, p.y)) for p in solution) / len(solution) if solution else 1.0
                        eta = avg ** beta
                        weights.append((cell, tau * eta))

                    total = sum(w for _, w in weights)
                    r = random.random() * total
                    cum = 0.0
                    for cell, w in weights:
                        cum += w
                        if r <= cum:
                            chosen = cell
                            break

                    pt = Point()
                    pt.x, pt.y, pt.z = chosen[0], chosen[1], self.flight_altitude
                    solution.append(pt)
                    available.discard(chosen)

                # 🔒 Bağlantı kontrolü
                if not is_connected(solution, self.comm_threshold):
                    continue  # bağlantısız çözüm, atla

                cost = self.fitness(solution)
                if cost < best_cost:
                    best_cost, best_sol = cost, solution

            # Feromon güncelle
            for c in pheromone:
                pheromone[c] *= (1 - evaporation)
            if best_sol:
                for pt in best_sol:
                    nearest = min(cells, key=lambda c: math.dist(c, (pt.x, pt.y)))
                    pheromone[nearest] += 1.0 / (1.0 + best_cost)

        return best_sol

    def sma_distribute(self):
        n = len(self.uavs)
        num_agents = 20
        iters = 50
        max_vel = 5.0
        perception_radius = self.comm_threshold

        agents = []
        velocities = []

        for _ in range(num_agents):
            agent = []
            velocity = []
            for _ in range(n):
                x = random.uniform(self.scan_x_min, self.scan_x_max)
                y = random.uniform(self.scan_y_min, self.scan_y_max)
                pt = Point(x=x, y=y, z=self.flight_altitude)
                agent.append(pt)
                velocity.append(Point(x=0.0, y=0.0, z=0.0))
            if is_connected(agent, self.comm_threshold):
                agents.append(agent)
                velocities.append(velocity)

        best_sol = None
        best_cost = float('inf')

        for _ in range(iters):
            for i in range(len(agents)):
                new_agent = []
                for j in range(n):
                    sep, ali, coh = Point(), Point(), Point()
                    neighbors = [agents[i][k] for k in range(n) if k != j and math.dist((agents[i][j].x, agents[i][j].y), (agents[i][k].x, agents[i][k].y)) < perception_radius]
                    if neighbors:
                        for nb in neighbors:
                            sep.x += agents[i][j].x - nb.x
                            sep.y += agents[i][j].y - nb.y
                            ali.x += velocities[i][j].x
                            ali.y += velocities[i][j].y
                            coh.x += nb.x
                            coh.y += nb.y
                        count = len(neighbors)
                        sep.x /= count
                        sep.y /= count
                        ali.x /= count
                        ali.y /= count
                        coh.x = (coh.x / count) - agents[i][j].x
                        coh.y = (coh.y / count) - agents[i][j].y

                    vx = velocities[i][j].x + sep.x + ali.x + coh.x
                    vy = velocities[i][j].y + sep.y + ali.y + coh.y

                    vx = max(min(vx, max_vel), -max_vel)
                    vy = max(min(vy, max_vel), -max_vel)

                    nx = agents[i][j].x + vx
                    ny = agents[i][j].y + vy
                    new_agent.append(Point(x=nx, y=ny, z=self.flight_altitude))
                    velocities[i][j] = Point(x=vx, y=vy, z=0.0)

                if is_connected(new_agent, self.comm_threshold):
                    agents[i] = new_agent
                    cost = self.fitness(new_agent)
                    if cost < best_cost:
                        best_cost = cost
                        best_sol = [Point(x=p.x, y=p.y, z=p.z) for p in new_agent]

        if best_sol is None:
            best_sol = [Point(
                x=self.final_target.x + u.formation_offset.x + random.uniform(-10, 10),
                y=self.final_target.y + u.formation_offset.y + random.uniform(-10, 10),
                z=self.flight_altitude
            ) for u in self.uavs]

        for i, u in enumerate(self.uavs):
            if math.dist((best_sol[i].x, best_sol[i].y), (u.position.x, u.position.y)) < 10.0:
                best_sol[i].x += random.uniform(15, 25)
                best_sol[i].y += random.uniform(15, 25)

        return best_sol


    def gwo_distribute(self):
            n = len(self.uavs)
            num_wolves = 20
            max_iter = 50
            wolves = []

            # Başlangıç: Bağlantılı kurtlar üret
            while len(wolves) < num_wolves:
                wolf = []
                for _ in range(n):
                    x = random.uniform(self.scan_x_min, self.scan_x_max)
                    y = random.uniform(self.scan_y_min, self.scan_y_max)
                    wolf.append(Point(x=x, y=y, z=self.flight_altitude))
                if is_connected(wolf, self.comm_threshold):
                    wolves.append(wolf)

            alpha, beta, delta = None, None, None
            alpha_score = beta_score = delta_score = float('inf')

            for t in range(max_iter):
                for wolf in wolves:
                    if not is_connected(wolf, self.comm_threshold):
                        continue

                    score = self.fitness(wolf)
                    if score < alpha_score:
                        delta, delta_score = beta, beta_score
                        beta, beta_score = alpha, alpha_score
                        alpha, alpha_score = [Point(x=p.x, y=p.y, z=p.z) for p in wolf], score
                    elif score < beta_score:
                        delta, delta_score = beta, beta_score
                        beta, beta_score = [Point(x=p.x, y=p.y, z=p.z) for p in wolf], score
                    elif score < delta_score:
                        delta, delta_score = [Point(x=p.x, y=p.y, z=p.z) for p in wolf], score

                if alpha is None:
                    continue  # henüz çözüm yok

                a = 2 - t * (2 / max_iter)
                for i in range(len(wolves)):
                    new_wolf = []
                    for j in range(n):
                        A1 = 2 * a * random.random() - a
                        A2 = 2 * a * random.random() - a
                        A3 = 2 * a * random.random() - a
                        C1 = 2 * random.random()
                        C2 = 2 * random.random()
                        C3 = 2 * random.random()

                        X1_x = alpha[j].x - A1 * abs(C1 * alpha[j].x - wolves[i][j].x)
                        X2_x = beta[j].x  - A2 * abs(C2 * beta[j].x  - wolves[i][j].x)
                        X3_x = delta[j].x - A3 * abs(C3 * delta[j].x - wolves[i][j].x)
                        new_x = (X1_x + X2_x + X3_x) / 3

                        X1_y = alpha[j].y - A1 * abs(C1 * alpha[j].y - wolves[i][j].y)
                        X2_y = beta[j].y  - A2 * abs(C2 * beta[j].y  - wolves[i][j].y)
                        X3_y = delta[j].y - A3 * abs(C3 * delta[j].y - wolves[i][j].y)
                        new_y = (X1_y + X2_y + X3_y) / 3

                        # Alan sınırları içinde tut
                        new_x = max(min(new_x, self.scan_x_max), self.scan_x_min)
                        new_y = max(min(new_y, self.scan_y_max), self.scan_y_min)

                        new_wolf.append(Point(x=new_x, y=new_y, z=self.flight_altitude))

                    if is_connected(new_wolf, self.comm_threshold):
                        wolves[i] = new_wolf

            # Alpha çözümsüz kaldıysa yedek üret
            if alpha is None or not is_connected(alpha, self.comm_threshold):
                self.get_logger().warn("GWO: Geçerli alpha çözümü bulunamadı, fallback hedefleri atanıyor.")
                alpha = []
                for u in self.uavs:
                    pt = Point()
                    pt.x = u.position.x + random.uniform(30, 50)
                    pt.y = u.position.y + random.uniform(30, 50)
                    pt.z = self.flight_altitude
                    alpha.append(pt)

            # Hedefler pozisyona çok yakınsa ittir
            for i, u in enumerate(self.uavs):
                dist = math.dist((alpha[i].x, alpha[i].y), (u.position.x, u.position.y))
                if dist < 2.0:
                    alpha[i].x = u.position.x + random.uniform(10, 20)
                    alpha[i].y = u.position.y + random.uniform(10, 20)

            # DEBUG LOG
            for i, pt in enumerate(alpha):
                self.get_logger().info(f"GWO Final Hedef UAV {i}: ({pt.x:.2f}, {pt.y:.2f})")

                

            return alpha




    def woa_distribute(self):
        n = len(self.uavs)
        num_whales = 20
        max_iter = 50
        whales = []
        for _ in range(num_whales):
            whale = [Point(
                x=random.uniform(self.scan_x_min, self.scan_x_max),
                y=random.uniform(self.scan_y_min, self.scan_y_max),
                z=self.flight_altitude) for _ in range(n)]
            if is_connected(whale, self.comm_threshold):
                whales.append(whale)

        best_whale = None
        best_score = float('inf')

        for t in range(max_iter):
            a = 2 - t * (2 / max_iter)
            for i in range(len(whales)):
                score = self.fitness(whales[i])
                if score < best_score:
                    best_whale = whales[i].copy()
                    best_score = score

            for i in range(len(whales)):
                new_positions = []
                for j in range(n):
                    r = random.random()
                    A = 2 * a * r - a
                    C = 2 * random.random()
                    l = random.uniform(-1, 1)
                    p = random.random()

                    if p < 0.5:
                        if abs(A) < 1:
                            D_x = abs(C * best_whale[j].x - whales[i][j].x)
                            D_y = abs(C * best_whale[j].y - whales[i][j].y)
                            new_x = best_whale[j].x - A * D_x
                            new_y = best_whale[j].y - A * D_y
                        else:
                            rand_whale = random.choice(whales)
                            D_x = abs(C * rand_whale[j].x - whales[i][j].x)
                            D_y = abs(C * rand_whale[j].y - whales[i][j].y)
                            new_x = rand_whale[j].x - A * D_x
                            new_y = rand_whale[j].y - A * D_y
                    else:
                        b = 1
                        dist = math.dist((best_whale[j].x, best_whale[j].y), (whales[i][j].x, whales[i][j].y))
                        new_x = dist * math.exp(b * l) * math.cos(2 * math.pi * l) + best_whale[j].x
                        new_y = dist * math.exp(b * l) * math.sin(2 * math.pi * l) + best_whale[j].y

                    new_positions.append(Point(x=new_x, y=new_y, z=self.flight_altitude))
                whales[i] = new_positions

        if best_whale is None:
            best_whale = [Point(
                x=self.final_target.x + u.formation_offset.x + random.uniform(-10, 10),
                y=self.final_target.y + u.formation_offset.y + random.uniform(-10, 10),
                z=self.flight_altitude) for u in self.uavs]

        for i, u in enumerate(self.uavs):
            if math.dist((best_whale[i].x, best_whale[i].y), (u.position.x, u.position.y)) < 10.0:
                best_whale[i].x += random.uniform(15, 25)
                best_whale[i].y += random.uniform(15, 25)

        return best_whale





    def cuckoo_distribute(self):
        import numpy as np

        # ───────── Levy flight yardımcıları ─────────
        def levy_flight(Lambda):
            sigma = (
                math.gamma(1 + Lambda) * math.sin(math.pi * Lambda / 2) /
                (math.gamma((1 + Lambda) / 2) * Lambda * 2 ** ((Lambda - 1) / 2))
            ) ** (1 / Lambda)
            u = np.random.normal(0, sigma, size=2)
            v = np.random.normal(0, 1, size=2)
            return u / abs(v) ** (1 / Lambda)

        def within_comm(x, y, cx, cy, r):
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            if d2 > r * r:
                scale = r / math.sqrt(d2)
                x = cx + (x - cx) * scale
                y = cy + (y - cy) * scale
            return x, y
        # ────────────────────────────────────────────

        n            = len(self.uavs)
        num_nests    = 20
        max_iter     = 50
        pa           = 0.25                 # tahrip olma olasılığı
        center_x     = (self.scan_x_min + self.scan_x_max) / 2
        center_y     = (self.scan_y_min + self.scan_y_max) / 2
        comm_range   = self.comm_threshold

        # ── Başlangıç: yalnızca bağlantılı yuvalar oluştur ──
        nests = []
        while len(nests) < num_nests:
            nest = []
            for _ in range(n):
                while True:
                    x = random.uniform(self.scan_x_min, self.scan_x_max)
                    y = random.uniform(self.scan_y_min, self.scan_y_max)
                    if math.dist((x, y), (center_x, center_y)) <= comm_range:
                        break
                nest.append(Point(x=x, y=y, z=self.flight_altitude))
            if is_connected(nest, comm_range):
                nests.append(nest)

        best_nest  = None
        best_score = float('inf')

        for _ in range(max_iter):
            new_nests = []

            # ── Levy-flight adımı ──
            for nest in nests:
                new_nest = []
                for pt in nest:
                    step      = levy_flight(1.5)
                    nx        = pt.x + step[0] * (pt.x - random.uniform(self.scan_x_min, self.scan_x_max))
                    ny        = pt.y + step[1] * (pt.y - random.uniform(self.scan_y_min, self.scan_y_max))
                    nx, ny    = within_comm(nx, ny, center_x, center_y, comm_range)
                    nx        = min(max(nx, self.scan_x_min), self.scan_x_max)
                    ny        = min(max(ny, self.scan_y_min), self.scan_y_max)
                    new_nest.append(Point(x=nx, y=ny, z=self.flight_altitude))
                # yalnızca bağlantılı yuvaları kabul et
                if is_connected(new_nest, comm_range):
                    new_nests.append(new_nest)
                else:
                    new_nests.append(nest)  # eskiyi koru

            # ── Değerlendir & iyileştir ──
            for i in range(num_nests):
                new_cost = self.fitness(new_nests[i])
                old_cost = self.fitness(nests[i])
                if new_cost < old_cost:
                    nests[i] = new_nests[i]
                    if new_cost < best_score:
                        best_score = new_cost
                        best_nest  = [Point(x=p.x, y=p.y, z=p.z) for p in new_nests[i]]

            # ── Rastgele yuva tahribi & yeniden oluşturma ──
            for i in range(num_nests):
                if random.random() < pa:
                    fresh = []
                    for _ in range(n):
                        while True:
                            x = random.uniform(self.scan_x_min, self.scan_x_max)
                            y = random.uniform(self.scan_y_min, self.scan_y_max)
                            if math.dist((x, y), (center_x, center_y)) <= comm_range:
                                break
                        fresh.append(Point(x=x, y=y, z=self.flight_altitude))
                    if is_connected(fresh, comm_range):
                        nests[i] = fresh

        return best_nest

    def timer_callback(self):
        dt = 0.1
        now = self.get_clock().now().nanoseconds / 1e9
        ma = MarkerArray()
        # hold → move
        if self.phase == "hold" and now - self.start_time >= self.hold_duration:
            self.phase = "move"
            for u in self.uavs:
                u.phase = "move"
            self.get_logger().info("Hold tamamlandı → move")

        # move → arrived
        if self.phase == "move":
            dx = self.final_target.x - self.global_form_center.x
            dy = self.final_target.y - self.global_form_center.y
            dz = self.final_target.z - self.global_form_center.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist > self.wp_thresh * len(self.uavs):
                step = self.speed * dt
                self.global_form_center.x += dx/dist * step
                self.global_form_center.y += dy/dist * step
                self.global_form_center.z += dz/dist * step
            else:
                self.phase = "arrived"
                self.global_form_center.x = self.final_target.x
                self.global_form_center.y = self.final_target.y
                self.global_form_center.z = self.final_target.z
                self.get_logger().info("Arrived → hesaplıyorum disperse")

            for u in self.uavs:
                if u.phase == "move":
                    tgt = Point()
                    tgt.x = self.global_form_center.x + u.formation_offset.x
                    tgt.y = self.global_form_center.y + u.formation_offset.y
                    tgt.z = self.global_form_center.z
                    u.update(dt, self.speed, self.wp_thresh, tgt)

        # arrived → disperse
        if self.phase == "arrived" and not self.disperse_initialized:

                self.disperse_initialized = True
                if self.algorithm == 1:
                    disps = self.pso_distribute()
                elif self.algorithm == 2:
                    disps = self.aco_distribute()
                elif self.algorithm == 3:
                    disps = self.sma_distribute()
                elif self.algorithm == 4:
                    disps = self.gwo_distribute()
                elif self.algorithm == 5:
                    disps = self.woa_distribute()
                elif self.algorithm == 6:
                    disps = self.cuckoo_distribute()

                self.get_logger().info("Disperse hedefleri:")
                for i, tgt in enumerate(disps):
                    self.get_logger().info(f"UAV {i}: ({tgt.x:.2f}, {tgt.y:.2f})")

                for u, tgt in zip(self.uavs, disps):
                    u.dispersion_target = tgt
                    u.phase = "disperse"

                self.phase = "disperse"
                self.start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info("Disperse hedefleri hazır → disperse")


        # disperse → final
        if self.phase == "disperse":
            done = True
            for u in self.uavs:
                if u.phase == "disperse":
                    u.update(dt, self.speed, self.wp_thresh, u.dispersion_target)
                    dx = abs(u.position.x - u.dispersion_target.x)
                    dy = abs(u.position.y - u.dispersion_target.y)
                    dz = abs(u.position.z - u.dispersion_target.z)
                    if dx > self.wp_thresh or dy > self.wp_thresh or dz > self.wp_thresh:
                        done = False

            # Her adımda yeni hedef üret (yaklaşarak dağılmayı sağla)
            if done:
                self.write_coverage_result()
                self.iteration_counter += 1
                self.remaining_iters -= 1

                if self.remaining_iters > 0:
                    self.get_logger().info(f"🔄 Yeni hedefler hesaplanıyor... ({self.remaining_iters} iterasyon kaldı)")
                    if self.algorithm == 1:
                        disps = self.pso_distribute()
                    elif self.algorithm == 2:
                        disps = self.aco_distribute()
                    elif self.algorithm == 3:
                        disps = self.sma_distribute()
                    elif self.algorithm == 4:
                        disps = self.gwo_distribute()
                    elif self.algorithm == 5:
                        disps = self.woa_distribute()
                    elif self.algorithm == 6:
                        disps = self.cuckoo_distribute()

                    for u, tgt in zip(self.uavs, disps):
                        u.dispersion_target = tgt
                        u.phase = "disperse"

                    self.start_time = self.get_clock().now().nanoseconds / 1e9
                    self.phase = "disperse"
                else:
                    self.get_logger().info("✅ Tüm iterasyonlar tamamlandı.")
                    self.phase = "final"


        self.publish_target_marker(ma)
        for u in self.uavs:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = "uav", u.id
            m.type, m.action = Marker.CUBE, Marker.ADD
            m.pose.position = u.position
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = 5.0, 5.0, 2.0
            m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0, 1.0
            m.lifetime.sec = 0  # Kalıcı olsun ama geçmişi silinsin
            m.action = Marker.ADD  # Var olan aynı ID'yi silip yenisini eklesin
            ma.markers.append(m)

            c = Marker()
            c.header, c.ns, c.id = m.header, "coverage", u.id+100
            comm = Marker()
            comm.header = m.header
            comm.ns = "comm_range"
            comm.id = u.id + 200
            comm.type = Marker.LINE_STRIP
            comm.action = Marker.ADD
            comm.pose.orientation.w = 1.0
            comm.scale.x = 0.8  # Çizgi kalınlığı

            comm.color.a = 1.0
            comm.color.r = 1.0  # Kırmızı
            comm.color.g = 0.0
            comm.color.b = 0.0

            points = []
            for i in range(37):
                ang = 2 * math.pi * i / 36
                p = Point()
                p.x = u.position.x + self.comm_threshold * math.cos(ang)
                p.y = u.position.y + self.comm_threshold * math.sin(ang)
                p.z = u.position.z
                points.append(p)
            comm.points = points
            ma.markers.append(comm)

            c.type, c.action = Marker.LINE_STRIP, Marker.ADD
            c.pose.orientation.w = 1.0
            c.scale.x = 1.0
            c.color.a, c.color.r, c.color.g, c.color.b = 0.5, 0.0, 1.0, 0.0
            pts = []
            for i in range(37):
                ang = 2*math.pi*i/36
                p = Point()
                p.x = u.position.x + u.coverage_radius*math.cos(ang)
                p.y = u.position.y + u.coverage_radius*math.sin(ang)
                p.z = u.position.z
                pts.append(p)
            c.points = pts
            ma.markers.append(c)

            gps = NavSatFix()
            gps.header.frame_id = "gps"
            gps.header.stamp = self.get_clock().now().to_msg()
            gps.latitude  = self.ref_lat + u.position.x/111320.0
            gps.longitude = self.ref_lon + u.position.y/(111320.0*math.cos(math.radians(self.ref_lat)))
            gps.altitude  = self.ref_alt + u.position.z
            gps.status.status  = NavSatStatus.STATUS_FIX
            gps.status.service = NavSatStatus.SERVICE_GPS
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.gps_pubs[u.id].publish(gps)
        self.get_logger().info(f"{len(ma.markers)} marker RViz'e gönderiliyor.")
        self.marker_pub.publish(ma)
        self.check_communications()
    def publish_target_marker(self, ma: MarkerArray):
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = "map"
        clear.header.stamp = self.get_clock().now().to_msg()
        ma.markers.append(clear)

        cx, cy, cz = self.final_target.x, self.final_target.y, self.final_target.z
        hx = (self.scan_x_max - self.scan_x_min) / 2.0
        hy = (self.scan_y_max - self.scan_y_min) / 2.0
        hz = self.flight_altitude / 2.0
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    p = Point()
                    p.x = cx + sx*hx
                    p.y = cy + sy*hy
                    p.z = cz + sz*hz
                    corners.append(p)
        edges = [
            (0,1),(1,3),(3,2),(2,0),
            (4,5),(5,7),(7,6),(6,4),
            (0,4),(1,5),(2,6),(3,7)
        ]
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "target", 888+random.randint(0, 10000)
        m.type, m.action = Marker.LINE_LIST, Marker.ADD
        m.scale.x = 1.0
        m.color.a, m.color.r, m.color.g, m.color.b = 0.8, 1.0, 1.0, 0.0
        pts = []
        for i, j in edges:
            pts.extend([corners[i], corners[j]])
        m.points = pts
        ma.markers.append(m)
    def check_communications(self):
        for s in self.uavs:
            for r in self.uavs:
                if s.id == r.id: continue
                d = math.dist((s.position.x,s.position.y),(r.position.x,r.position.y))
                tag = "sent" if d <= self.comm_threshold else "NOT sent"
                self.get_logger().info(f"UAV {s.id} -> UAV {r.id}: Message {tag} ({d:.2f} m)")
    def calculate_coverage_percent(self):
        grid_size = 2.0
        nx = int((self.scan_x_max - self.scan_x_min) / grid_size)
        ny = int((self.scan_y_max - self.scan_y_min) / grid_size)
        total_cells = nx * ny
        covered = set()

        for i in range(nx):
            for j in range(ny):
                x = self.scan_x_min + (i + 0.5) * grid_size
                y = self.scan_y_min + (j + 0.5) * grid_size
                for u in self.uavs:
                    d = math.dist((x, y), (u.position.x, u.position.y))
                    if d <= u.coverage_radius:
                        covered.add((i, j))
                        break
        return covered, total_cells

    def write_coverage_result(self):
        new_cells, total_cells = self.calculate_coverage_percent()
        self.coverage_so_far |= new_cells  # birleşim kümesi
        percent = (len(self.coverage_so_far) / total_cells) * 100.0 if total_cells > 0 else 0.0

        log_txt_path = "/tmp/coverage_result.txt"
        log_csv_path = os.path.expanduser("~/Downloads/Proje3/Proje/Proje/ros2_ws/coverage_log.csv")
        os.makedirs(os.path.dirname(log_csv_path), exist_ok=True)

        # Basit log
        with open(log_txt_path, "w") as f:
            f.write(f"{percent:.2f}")

        headers = [
            "Zaman (saniye)", "UAV Sayısı", "Algoritma", "Formasyon",
            "Kapsama (%)", "Alan Genişliği", "İterasyon"
        ]
        duration = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        row = [
            f"{duration:.2f}", str(len(self.uavs)),
            str(self.algorithm),
            self.get_parameter("formation").value,
            f"{percent:.2f}",
            self.get_parameter("area_size").value,
            str(self.iteration_counter)
        ]

        is_new_file = not os.path.exists(log_csv_path)
        with open(log_csv_path, "a", newline='') as f:
            writer = csv.writer(f)
            if is_new_file:
                writer.writerow(headers)
            writer.writerow(row)

        self.get_logger().info(f"[İterasyon {self.iteration_counter}] CSV güncellendi: {percent:.2f}% kapsama")
       


   


def all_within_comm_range(points, comm_threshold):
    for i, p1 in enumerate(points):
        connected = False
        for j, p2 in enumerate(points):
            if i != j and math.dist((p1.x, p1.y), (p2.x, p2.y)) <= comm_threshold:
                connected = True
                break
        if not connected:
            return False
    return True

def main():
    rclpy.init()
    sim = UAVSimulator()
    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass
    sim.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
