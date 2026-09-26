EPS = 1e-6

def is_equal(x, y):
    """Return True when x and y should be treated as equal."""
    return abs(x - y) <= EPS

def is_less(x, y):
    """Return True when x is definitely smaller than y."""
    return x < y - EPS

def is_greater(x, y):
    """Return True when x is definitely greater than y."""
    return x > y + EPS
    
def is_less_equal(x, y):
    """Return True when x should be treated as less than or equal to y."""
    return not is_greater(x, y)
    
def is_greater_equal(x, y):
    """Return True when x should be treated as greater than or equal to y."""
    return not is_less(x, y)

class Operation:
    __slots__ = ['x', 'y', 'z']
    
    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        
    def __add__(self, other):
        return Operation(self.x + other.x, self.y + other.y, self.z + other.z)
        
    def __sub__(self, other):
        return Operation(self.x - other.x, self.y - other.y, self.z - other.z)
        
    def __mul__(self, s):
        return Operation(self.x * s, self.y * s, self.z * s)
        
    def magnitude(self):
        return (self.x**2 + self.y**2 + self.z**2)**0.5
    
    def is_zero(self):
        return is_equal(self.magnitude(), 0.0)
    
    def __eq__(self, other):
        if not isinstance(other, Operation):
            return False
        return is_equal(self.x, other.x) and is_equal(self.y, other.y) and is_equal(self.z, other.z)
        
    def __hash__(self):
        return hash((self.x, self.y, self.z))

def ground(position, velocity, acceleration):
    if is_equal(acceleration, 0.0):
        if is_equal(velocity, 0.0):
            if is_equal(position, 0.0):
                return [0.0]
            else:
                return []
        t = -position / velocity
        if is_greater_equal(t, 0.0):
            return [max(0.0, t)]
        else:
            return []
            
    D = (velocity**2) - (2 * acceleration * position)
    
    if is_less(D, 0.0):
        return []
    
    d = (max(0.0, D))**0.5
    t1 = (-velocity + d) / acceleration
    t2 = (-velocity - d) / acceleration
    ans = []
    for i in (t1, t2):
        if is_greater_equal(i, 0.0):
            ans.append(max(0.0, i))
    return ans

class Flight:
    """A single aircraft."""
    
    def __init__(self, max_speed, position, velocity, time=0.0):
        self.mx = float(max_speed)
        self.p = Operation(*position)
        self.v = Operation(*velocity)
        self.a = Operation(0, 0, 0)
        self.t = float(time)
        self.state = 'flying'

    def future(self, time):
        if self.state != 'flying':
            return self.state
        
        cht = time - self.t
        
        if is_less_equal(cht, 0.0):
            if is_greater(self.v.magnitude(), self.mx):
                return 'accident'
            if is_equal(self.p.z, 0.0) and is_less(self.v.z, 0.0):
                return 'accident'
            if is_equal(self.p.z, 0.0) and is_equal(self.v.z,0.0) and is_greater(self.a.z,0.0):
                return 'flying'
            if is_equal(self.p.z, 0.0) and is_equal(self.v.z,0.0) and is_less(self.a.z,0.0):
                return 'accident'
            if is_equal(self.p.z, 0.0) and self.v.is_zero() and self.a.is_zero():
                return 'landed safely'
            return 'flying'
        
        gt = ground(self.p.z, self.v.z, self.a.z)
        for t in gt:
            if is_less_equal(t, cht):
                v_at_t = self.v + self.a * t
                if is_less(v_at_t.z, 0.0) or is_less(self.a.z, 0.0):
                    return 'accident'
                if v_at_t.is_zero() and self.a.is_zero():
                    return 'landed safely'
                    
        vf = self.v + self.a * cht
        if is_greater(vf.magnitude(), self.mx):
            return 'accident'
        return 'flying'
        
    def set_acceleration(self, time, acceleration):
        self.status(time)
        if self.state == 'flying':
            self.a = Operation(*acceleration)
        return self.state
        
    def status(self, time):
        a = self.future(time)
        if a != 'flying':
            self.state = a
            return self.state
        
        cht = time - self.t
        if is_greater(cht, 0.0):
            self.p = self.p + (self.v * cht) + (self.a * (0.5 * (cht**2)))
            self.v = self.v + (self.a * cht)
            self.t = float(time)
            
        return self.state


class AirTrafficControl:
    """The controller, which owns every flight."""
    
    def __init__(self):
        self.dic = {}
        self.cl = 0.0
        
    def coll(self):
        active = [plane for plane in self.dic.values() if plane.state == 'flying']
        mn = float('inf')
        c = set()
        
        for i in range(len(active)):
            for j in range(i+1, len(active)):
                p1, p2 = active[i], active[j]
                rp = p1.p - p2.p
                rv = p1.v - p2.v
                ra = p1.a - p2.a
                
               
                if ra.is_zero() and rv.is_zero() and rp.is_zero():
                    ans = 0.0
                else:
                    l = ground(rp.x, rv.x, ra.x) + ground(rp.y, rv.y, ra.y) + ground(rp.z, rv.z, ra.z)
                    ans = float('inf')
                    for t in l:
                        f = rp + (rv * t) + (ra * (0.5 * (t**2)))
                        if f.is_zero():
                            ans = min(ans, t)
                            
                if is_equal(ans, mn):
                    c.update({p1, p2})
                elif is_less(ans, mn):
                    mn = ans
                    c = {p1, p2}
                    
        return mn, c
        
    def simulation(self, time):
        ct, planes = self.coll()
        
        if is_less_equal(self.cl + ct, time):
            ti = self.cl + ct
            
            for p in self.dic.values():
                if p.state == 'flying':
                    p.status(ti)
                    
            self.cl = ti
            
            planes_list = list(planes)
            to_crash = set()
            
            for i in range(len(planes_list)):
                for j in range(i+1, len(planes_list)):
                    p1, p2 = planes_list[i], planes_list[j]
                    if p1.state == 'flying' and p2.state == 'flying':
                        if p1.p == p2.p:
                            to_crash.add(p1)
                            to_crash.add(p2)
 
            for p in to_crash:
                p.state = 'accident'
                
            return True
            
        else:
            self.cl = time
            for p in self.dic.values():
                if p.state == 'flying':
                    p.status(self.cl)
            return False
            
    def forward(self, time):
        fl = True
        while fl or is_less(self.cl, time):
            fl = self.simulation(time)
            
    def create(self, time, flight_id, max_speed, position, velocity):
        self.forward(time)
        if flight_id not in self.dic:
            f = Flight(max_speed, position, velocity, time)
            f.status(time)  
            self.dic[flight_id] = f
            
    def update(self, time, flight_id, acceleration):
        self.forward(time)
        if flight_id in self.dic:
            self.dic[flight_id].set_acceleration(time, acceleration)
            
    def status(self, time, flight_id):
        self.forward(time)
        if flight_id not in self.dic:
            return 'does not exist'
        return self.dic[flight_id].status(time)
