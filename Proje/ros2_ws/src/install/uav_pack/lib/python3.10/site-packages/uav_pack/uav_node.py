#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
import math
import random

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
        if self.phase in ("move", "disperse"):
            dx = target.x - self.position.x
            dy = target.y - self.position.y
            dz = target.z - self.position.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist < thresh:
                self.position = target
            else:
                step = speed * dt
                self.position.x += dx/dist * step
                self.position.y += dy/dist * step
                self.position.z += dz/dist * step

class UAVSimulator(Node):
    def __init__(self):
        super().__init__('uav_simulation')

        # 1) Parametreleri oku
        self.declare_parameter('num_uav',   5)
        self.declare_parameter('formation', 1)
        self.declare_parameter('algorithm', 1)
        n    = self.get_parameter('num_uav').value
        ft   = self.get_parameter('formation').value
        algo = self.get_parameter('algorithm').value
        self.algorithm = algo
        self.get_logger().info(f"Parametreler: num_uav={n}, formation={ft}, algorithm={algo}")

        # 2) Referans ve scan alanı
        self.ref_lat, self.ref_lon, self.ref_alt = 41.0, 29.0, 0.0
        for p,v in [
            ('scan_min_lat',41.0), ('scan_max_lat',41.00054),
            ('scan_min_lon',29.0), ('scan_max_lon',29.000714),
            ('scan_min_alt',0.0),   ('scan_max_alt',20.0),
        ]:
            self.declare_parameter(p, v)
        min_lat = self.get_parameter('scan_min_lat').value
        max_lat = self.get_parameter('scan_max_lat').value
        min_lon = self.get_parameter('scan_min_lon').value
        max_lon = self.get_parameter('scan_max_lon').value
        min_alt = self.get_parameter('scan_min_alt').value
        max_alt = self.get_parameter('scan_max_alt').value

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
        self.hold_center.x = self.final_target.x + 50.0
        self.hold_center.y = self.final_target.y
        self.hold_center.z = self.flight_altitude

        # 6) Global formasyon merkezi (hold_center’ın kopyası)
        self.global_form_center = Point()
        self.global_form_center.x = self.hold_center.x
        self.global_form_center.y = self.hold_center.y
        self.global_form_center.z = self.hold_center.z

        # 7) UAV parametreleri
        for p,v in [
            ('coverage_radius',10.0),
            ('comm_threshold',20.0),
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
        dmin = min(
            math.dist((p1.x,p1.y),(p2.x,p2.y))
            for i,p1 in enumerate(pts) for p2 in pts[i+1:]
        )
        cost = -dmin
        if not is_connected(pts, self.comm_threshold):
            cost += 1e5
        return cost

    def pso_distribute(self):
        n = len(self.uavs)
        iters = 50
        w,c1,c2 = 0.7,1.5,1.5

        # Başlangıç pozisyon ve hız
        pos = []
        vel = []
        for u in self.uavs:
            p = Point()
            p.x, p.y, p.z = u.position.x, u.position.y, u.position.z
            pos.append(p)
            v = Point()
            v.x = v.y = v.z = 0.0
            vel.append(v)

        # pbest & gbest
        pbest = []
        for p in pos:
            pb = Point()
            pb.x, pb.y, pb.z = p.x, p.y, p.z
            pbest.append(pb)
        pbest_cost = [self.fitness(pos)] * n

        gbest = Point()
        gbest.x, gbest.y, gbest.z = pbest[0].x, pbest[0].y, pbest[0].z
        gbest_cost = pbest_cost[0]

        # PSO döngüsü
        for _ in range(iters):
            for i in range(n):
                for coord in ('x','y'):
                    r1, r2 = random.random(), random.random()
                    pi = getattr(pbest[i], coord)
                    ci = getattr(pos[i], coord)
                    gi = getattr(gbest, coord)
                    vi = (w * getattr(vel[i], coord)
                          + c1 * r1 * (pi - ci)
                          + c2 * r2 * (gi - ci))
                    setattr(vel[i], coord, vi)

                pos[i].x = min(max(pos[i].x + vel[i].x, self.scan_x_min), self.scan_x_max)
                pos[i].y = min(max(pos[i].y + vel[i].y, self.scan_y_min), self.scan_y_max)
                pos[i].z = self.flight_altitude

                cost = self.fitness(pos)
                if cost < pbest_cost[i]:
                    pbest_cost[i] = cost
                    pbest[i].x, pbest[i].y, pbest[i].z = pos[i].x, pos[i].y, pos[i].z
                if cost < gbest_cost:
                    gbest_cost = cost
                    gbest.x, gbest.y, gbest.z = pos[i].x, pos[i].y, pos[i].z

        # Sonuç + formation_offset
        result = []
        for u in self.uavs:
            p = Point()
            p.x = gbest.x + u.formation_offset.x
            p.y = gbest.y + u.formation_offset.y
            p.z = self.flight_altitude
            result.append(p)
        return result

    def aco_distribute(self):
            """
            Güncellenmiş ACO:
            - Tüm konumlar küpün içinde.
            - Her eklenen UAV, mutlaka önceki çözümdeki bir UAV ile ≤ comm_threshold mesafede.
            - Bitmiş çözümün tüm UAV’leri multi‐hop bağlantıda kalacak şekilde üretilir.
            """
            num_ants    = 20
            num_iters   = 30
            evaporation = 0.1
            alpha       = 1.0
            beta        = 2.0

            # 1) Kare ızgara: sadece küpün içini tarasın
            grid_x = [self.scan_x_min + (i+0.5)*(self.scan_x_max-self.scan_x_min)/10 for i in range(10)]
            grid_y = [self.scan_y_min + (j+0.5)*(self.scan_y_max-self.scan_y_min)/10 for j in range(10)]
            cells = [(x,y) for x in grid_x for y in grid_y]

            pheromone = {c: 1.0 for c in cells}
            best_sol, best_cost = None, float('inf')

            for _ in range(num_iters):
                for _ in range(num_ants):
                    solution = []
                    available = set(cells)

                    for k in range(len(self.uavs)):
                        # k==0 ise tüm hücreler, değilse en az bir öncekiyle doğrudan bağlantıda olanlar
                        if k == 0:
                            candidates = list(available)
                        else:
                            candidates = [
                                c for c in available
                                if any(
                                    math.dist(c, (p.x,p.y)) <= self.comm_threshold
                                    for p in solution
                                )
                            ]
                            if not candidates:
                                candidates = list(available)

                        # feromon * sezgisel ağırlık
                        weights = []
                        for cell in candidates:
                            tau = pheromone[cell]**alpha
                            if solution:
                                avg = sum(math.dist(cell, (p.x,p.y)) for p in solution) / len(solution)
                            else:
                                avg = 1.0
                            eta = avg**beta
                            weights.append((cell, tau * eta))

                        total = sum(w for _,w in weights)
                        r = random.random() * total
                        cum = 0.0
                        for cell, w in weights:
                            cum += w
                            if r <= cum:
                                chosen = cell
                                break

                        # seçilen hücreyi Point’e dönüştür
                        pt = Point()
                        pt.x, pt.y, pt.z = chosen[0], chosen[1], self.flight_altitude
                        solution.append(pt)
                        available.discard(chosen)

                    # tam bağlantı kontrolü
                    if not is_connected(solution, self.comm_threshold):
                        cost = float('inf')
                    else:
                        cost = self.fitness(solution)

                    if cost < best_cost:
                        best_cost, best_sol = cost, solution

                # feromon buharlaşması
                for c in pheromone:
                    pheromone[c] *= (1 - evaporation)
                # en iyi çözüme ek feromon
                for pt in best_sol:
                    nearest = min(cells, key=lambda c: math.dist(c,(pt.x,pt.y)))
                    pheromone[nearest] += 1.0 / (1.0 + best_cost)

            # formation offset ile kaydırıp döndür
            final = []
            for u, pt in zip(self.uavs, best_sol):
                p = Point()
                p.x = pt.x + u.formation_offset.x
                p.y = pt.y + u.formation_offset.y
                p.z = self.flight_altitude
                final.append(p)
            return final

     


    def timer_callback(self):
        dt = 0.1
        now = self.get_clock().now().nanoseconds / 1e9

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
            if dist > self.wp_thresh:
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
        if self.phase == "arrived":
            disps = self.pso_distribute() if self.algorithm==1 else self.aco_distribute()
            for u,tgt in zip(self.uavs, disps):
                u.dispersion_target = tgt
                u.phase = "disperse"
            self.phase = "disperse"
            self.get_logger().info("Disperse hedefleri hazır → disperse")

        # disperse → final
        if self.phase == "disperse":
            done = True
            for u in self.uavs:
                if u.phase == "disperse":
                    u.update(dt, self.speed, self.wp_thresh, u.dispersion_target)
                    if (abs(u.position.x - u.dispersion_target.x) > self.wp_thresh or
                        abs(u.position.y - u.dispersion_target.y) > self.wp_thresh):
                        done = False
                    else:
                        u.phase = "final"
            if done:
                self.phase = "final"
                self.get_logger().info("Disperse tamamlandı → final")

        # Marker & GPS
        ma = MarkerArray()
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
            ma.markers.append(m)

            c = Marker()
            c.header, c.ns, c.id = m.header, "coverage", u.id+100
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

        self.marker_pub.publish(ma)
        self.check_communications()

    def publish_target_marker(self, ma: MarkerArray):
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
        m.ns, m.id = "target", 888
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

