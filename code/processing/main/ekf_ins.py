
class ekf_ins(object):
	"""docstring for ekf_ins"""
	def __init__(self,time, vn,ve,vd,lat,lon,alt,p,q,r,ax,ay,az,hx,hy,hz):
		super(ekf_ins, self).__init__()
		self.time = time
		self.vn = vn
		self.ve = ve
		self.vd = vd
		self.lat = lat
		self.lon = lon
		self.alt = alt
		self.p = p
		self.q = q
		self.r = r
		self.ax = ax
		self.ay = ay
		self.az = az
		self.hx = hx
		self.hy = hy
		self.hz = hz

		self.gbx = p;
  		self.gby = q;
  		self.gbz = r;
		
  		theta,phi,psi = getPitchRollYaw(ax, ay, az, hx, hy, hz);
