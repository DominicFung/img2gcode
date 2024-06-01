import numpy as np
from math import *
import operator

prn_detail = 0
epsilon = 1e-5
epsilon16 = 1e-16
is_ddd1 = True
roughing_depth_delta = 0.2 

def cmp(a, b):
    return (a > b) - (a < b) 

def ball_tool(r,rad):
    if r == rad: return rad
    #s = -sqrt(rad**2-r**2)
    s = rad-sqrt(rad**2-r**2)
    return s

def endmill(r,dia):
    return 0

def vee_common(angle):
    #slope = tan(angle * pi / 180)
    slope = tan(radians((180-angle)/2) )
    def f(r, dia):
        return r * slope
    return f

try:
    import numpy as numarray
    import numpy.core
    olderr = numpy.core.seterr(divide='ignore')
    plus_inf = (np.array((1.,))/0.)[0]
    numpy.core.seterr(**olderr)
except ImportError:
    importingError = True

def circ(r,b): 
    """\
Calculate the portion of the arc to do so that none is above the
safety height (that's just silly)"""

    z = r**2 - (r-b)**2
    if z < 0: z = 0
    return z**.5

class ArcEntryCut:
    def __init__(self, feed, max_radius):
        self.feed = feed
        self.max_radius = max_radius

    def __call__(self, conv, i0, j0, points):
        if len(points) < 2:
            p = points[0][1]
            if self.feed:
                conv.g.set_feed(self.feed)
            conv.g.safety()
            conv.g.rapid(p[0], p[1])
            if self.feed:
                conv.g.set_feed(conv.feed)
            return

        p1 = points[0][1]
        p2 = points[1][1]
        z0 = p1[2]

        lim = int(ceil(self.max_radius / conv.pixelsize))
        r = range(1, lim)

        if self.feed:
            conv.g.set_feed(self.feed)
        conv.g.safety()

        x, y, z = p1

        pixelsize = conv.pixelsize
        
        cx = cmp(p1[0], p2[0])
        cy = cmp(p1[1], p2[1])

        radius = self.max_radius

        if cx != 0:     #if rows
            h1 = conv.h1
            for di in r:
                dx = di * pixelsize
                i = i0 + cx * di
                if i < 0 or i >= h1: break
                z1 = conv.get_z(i, j0)
                dz = (z1 - z0)
                if dz <= 0: continue
                if dz > dx:
                    conv.g.write("(case 1)")
                    radius = dx
                    break
                rad1 = (dx * dx / dz + dz) / 2
                if rad1 < radius:
                    radius = rad1
                if dx > radius:
                    break

            z1 = min(p1[2] + radius, conv.safetyheight)
            x1 = p1[0] + cx * circ(radius, z1 - p1[2])
            conv.g.rapid(x1, p1[1])
            conv.g.cut(z=z1)

            conv.g.flush(); conv.g.lastgcode = None
            if cx > 0:
                conv.g.write("G3 X%f Z%f R%f" % (p1[0], p1[2], radius))
            else:
                conv.g.write("G2 X%f Z%f R%f" % (p1[0], p1[2], radius))
            conv.g.lastx = p1[0]
            conv.g.lasty = p1[1]
            conv.g.lastz = p1[2]
        else:        #if cols
            w1 = conv.w1
            for dj in r:
                dy = dj * pixelsize
                j = j0 - cy * dj
                if j < 0 or j >= w1: break
                z1 = conv.get_z(i0, j)
                dz = (z1 - z0)
                if dz <= 0: continue
                if dz > dy:
                    radius = dy
                    break
                rad1 = (dy * dy / dz + dz) / 2
                if rad1 < radius: radius = rad1
                if dy > radius: break

            z1 = min(p1[2] + radius, conv.safetyheight)
            y1 = p1[1] + cy * circ(radius, z1 - p1[2])
            conv.g.rapid(p1[0], y1)
            conv.g.cut(z=z1)

            conv.g.flush(); conv.g.lastgcode = None
            if cy > 0:
                conv.g.write("G2 Y%f Z%f R%f" % (p1[1], p1[2], radius))
            else:
                conv.g.write("G3 Y%f Z%f R%f" % (p1[1], p1[2], radius))
            conv.g.lastx = p1[0]
            conv.g.lasty = p1[1]
            conv.g.lastz = p1[2]
        if self.feed:
            conv.g.set_feed(conv.feed)

class Convert_Scan_Increasing:
    def __call__(self, primary, items):
        yield True, items

    def reset(self):
        pass

class Convert_Scan_Decreasing:
    def __call__(self, primary, items):
        items.reverse()
        yield True, items

    def reset(self):
        pass

class Convert_Scan_Upmill:
    def __init__(self, slop = sin(pi / 18)):
        self.slop = slop

    def __call__(self, primary, items):
        for span in group_by_sign(items, self.slop, operator.itemgetter(2)):
            if amax([it[2] for it in span]) < 0:
                span.reverse()
            yield True, span

    def reset(self):
        pass

class Convert_Scan_Downmill:
    def __init__(self, slop = sin(pi / 18)):
        self.slop = slop

    def __call__(self, primary, items):
        for span in group_by_sign(items, self.slop, operator.itemgetter(2)):
            if amax([it[2] for it in span]) > 0:
                span.reverse()
            yield True, span

    def reset(self):
        pass

class Reduce_Scan_Lace:
    def __init__(self, converter, slope, keep):
        self.converter = converter
        self.slope = slope
        self.keep = keep

    def __call__(self, primary, items):
        slope = self.slope
        keep = self.keep
        if primary:
            idx = 3
            test = operator.le
        else:
            idx = 2
            test = operator.ge

        def bos(j):
            return j - j % keep

        def eos(j):
            if j % keep == 0: return j
            return j + keep - j%keep

        for i, (flag, span) in enumerate(self.converter(primary, items)):
            subspan = []
            a = None
            for i, si in enumerate(span):
                ki = si[idx]
                if a is None:
                    if test(abs(ki), slope):
                        a = b = i
                else:
                    if test(abs(ki), slope):
                        b = i
                    else:
                        if i - b < keep: continue
                        yield True, span[bos(a):eos(b+1)]
                        a = None
            if a is not None:
                yield True, span[a:]

    def reset(self):
        self.converter.reset()
  
class Convert_Scan_Alternating:
    def __init__(self):
        self.st = 0

    def __call__(self, primary, items):
        st = self.st = self.st + 1
        if st % 2: items.reverse()
        if st == 1: yield True, items
        else: yield False, items

    def reset(self):
        self.st = 0

def make_tool_shape(f, wdia, resp, is_offset= False, tool_type=0, wdia2=-9999, degr=60, units=0):
    res = 1. / resp
    
    # if cell_center:  =-> 'smooth terrain, but can cut the edge'; 
    # elif 'cell_contact' in other words 'Touch a cell at the nearest point': =-> 'more accurate'
    #
    # if cell_coeff ==  0 -   0% - the same as 'cell contact'; 
    # if cell_coeff ==2/3 -  33% - this is '33% from cell contact'; 
    # if cell_coeff ==  1 -  50% - this is 'cell center'; 
    # if cell_coeff ==  2 - 100% - this is 'the far corner of the cell'; This is MAX of cell_coeff.
    cell_coeff = 2./3.

    # that there - dia - is always an odd
    # because that more accurately
    always_an_odd = True

    dia = int(wdia*res)
    if dia/res < wdia: dia = dia +1
    if dia < 1: dia = 1
    
    if is_offset:
        if dia == 1: 
            dia = 2
            if prn_detail > 0: print("(Warning: offset[{0}] <= pixel size[{1}]! Use function correct_offset!)".format((wdia/2),resp))
            #this not enaf! Use function correct_offset - for correct offset!
            wdia = resp*2
    
    if always_an_odd and not dia % 2: dia = dia +1

    if is_offset and prn_detail > 0: print("(Real offset = {0} dia: {1} pixels)".format((dia*resp/2),dia))

    wrad = wdia/2.
    wrad0= wrad

    if wdia2 > wdia and degr > 0 and degr < 180:
        degr2 = (180-degr)/2
        
        f2 = vee_common(degr)
        if tool_type == 0:
            wrad0 = cos(radians(90-degr2))*wrad

            if wrad0 > wrad or wrad0 == 0 and prn_detail > -1: print("( Error tool2(2): ",wrad0, " )")
            if prn_detail > 1: print("( Radius of Ball End: {0:.2f} )".format(wrad0))

        if   tool_type == 0: h0 = (wrad - sin(radians(90-degr2))*wrad) / sin(radians(90-degr2))
        elif tool_type == 1: h0 = wrad/tan(radians(degr/2))
        elif tool_type == 2: h0 = f2(wrad, wrad) - f(wrad, wrad)
        elif tool_type == 3: h0 = f2(wrad, wrad) - f(wrad, wrad)
        elif tool_type == 4: h0 = f2(wrad, wrad) - f(wrad, wrad)
        elif tool_type == 5: h0 = f2(wrad, wrad) - f(wrad, wrad)

        wrad2 = wdia2/2.
        dia2= int(wdia2*res+.5)
        if dia2/res < wdia2: dia2 = dia2 +1
    
        if always_an_odd and not dia2 % 2: dia2 = dia2 +1

        dia = max(dia,dia2)
    else:
        wrad2 = -plus_inf

    #n = np.array([[plus_inf] * dia] * dia, type="Float32")
    n = np.array([[plus_inf] * dia] * dia, dtype=np.float32)
    hdia = dia / 2.
    #hdia = int(hdia)
    l = []

    for x in range(dia):
        for y in range(dia):
            if dia % 2 and x == dia/2 and y == dia/2:
                z = f(0, wrad)
                l.append(z)
                if z != 0.: 
                    if prn_detail > -1: print("( Error(0) tool center mast be = 0, bat z=",z, " )")
                    z = 0 
                n[x,y] = z  # z = 0 - mast be
            else:

                x1 = x-hdia
                y1 = y-hdia
                dopx2 = 0.
                dopy2 = 0.

                if x1 <= 0. and y1 < 0.:
                    if x1 == 0.: dopx = 0.
                    elif x1 >= -.5: dopx = 0.5
                    else: 
                        dopx = 1
                        dopx2 = -0.5

                    if y1 == 0.: dopy = 0.
                    elif y1 >= -.5: dopy = 0.5
                    else: 
                        dopy = 1
                        dopy2 = -0.5
                elif x1 <= 0.:
                    if x1 == 0.: dopx = 0.
                    elif x1 >= -.5: dopx = 0.5
                    else: 
                        dopx = 1
                        dopx2 = -0.5
                    dopy = 0
                    dopy2 = 0.5
                elif y1 < 0.:
                    dopx = 0
                    dopx2 = 0.5
                    if y1 == 0.: dopy = 0.
                    elif y1 >= -.5: dopy = 0.5
                    else: 
                        dopy = 1
                        dopy2 = -0.5
                else:
                    dopx = 0
                    dopy = 0
                    dopx2 = 0.5
                    dopy2 = 0.5

                dopx2 *= cell_coeff
                dopy2 *= cell_coeff
            
                r = hypot(x1+dopx+dopx2,   y1+dopy+dopy2)   * resp

                if r < wrad0:
                    z = f(r, wrad)
                    l.append(z)
                    n[x,y] = z
                    if z < 0. and prn_detail > -1: print("( tool error 1: r=",r," x=",x," y=",y," hight<0 hight= ",z,", mast be >= 0 )")
                    #if z > 300: print(" error 1: tool hight1: r=",r," x=",x," y=",y," hight= ",z," is too big!")
                elif r < wrad2:
                    z = f2(r,wrad2) - h0
                    l.append(z)
                    n[x,y] = z
                    if z < 0. and prn_detail > -1: print("( tool error 2: r=",r," x=",x," y=",y," z<0 z= ",z,", mast be >= 0 )")
                    #if z > 300: print(" error 2: tool hight1: r=",r," x=",x," y=",y," hight= ",z," is too big!")

    if n.min() != 0. and prn_detail > -1: print("( error(0): tool.minimum = ",n.min(),", mast be == 0 )")
    return n

def correct_offset(offset, resp):
    if offset > epsilon and offset < resp: 
        if prn_detail > -1: print("(Warning: offset[{0}] <= pixel size[{1}]. New offset = {1} units)".format(offset,resp))
        offset = resp
    return offset

def optim_tool_shape(n,hig,offset,tolerance):

    #return n

    lineLF = -1
    minH = hig+offset+tolerance+.5
    P = True
    dia = len(n)
    while P and lineLF < (dia-1):
        lineLF = lineLF + 1
        #left
        for x in range(dia):
            if n[lineLF,x] < minH:
                P = False
                break

        if P:
            #up
            for y in range(dia):
                if n[y,lineLF] < minH:
                    P = False
                    break
    #@ dia = 3, lineLF = 1

    #lineLF = lineLF - 1

    if lineLF > 0:


        P = True
        line2 = dia
        while P and line2 >= lineLF:
            line2 = line2 - 1
            #right
            for x in range(dia):
                if n[line2,x] < minH:
                    P = False
                    break
            if P:
                #down
                for y in range(dia):
                    if n[y,line2] < minH:
                        P = False
                        break
        #@ dia = 3, line2 = 1
        line2 = line2 + 1

        if prn_detail > 1: print("( tool opitimize - dia = ",dia," lines:)")
        if prn_detail > 1: print("( cut lines[left  and up  ]: ",lineLF,")")
        if prn_detail > 1: print("( cut lines[right and down]: ",(dia-line2),")")

        if line2 < dia:
            dia2 = line2 - lineLF
            n2 = np.array([[plus_inf] * dia2] * dia2, dtype=np.float32)
            d2range = range(lineLF,line2)
            for x in d2range:
                for y in d2range:
                    n2[x-lineLF,y-lineLF] = n[x,y]
        else:
            dia2 = dia - lineLF
            n2 = np.array([[plus_inf] * dia2] * dia2, dtype=np.float32)
            d2range = range(lineLF,dia)
            for x in d2range:
                for y in d2range:
                    n2[x-lineLF,y-lineLF] = n[x,y]
        if len(n2) == 0 and prn_detail > -1: print("( Error(): ",dia,lineLF,line2," )")
        if n2.min() != 0 and prn_detail > -1: print("( Error(): n2=",n2," )")
        return n2
    else:
        return n

def amax(seq):
    res = 0
    for i in seq:
        if abs(i) > abs(res): res = i
    return res

def group_by_sign(seq, slop=sin(pi/18), key=lambda x:x):
    sign = None
    subseq = []
    for i in seq:
        ki = key(i)
        if sign is None:
            subseq.append(i)
            if ki != 0:
                sign = ki / abs(ki)
        else:
            subseq.append(i)
            if sign * ki < -slop:
                sign = ki / abs(ki)
                yield subseq
                subseq = [i]
    if subseq: yield subseq

def ball_tool(r,rad):
    if r == rad: return rad
    #s = -sqrt(rad**2-r**2)
    s = rad-sqrt(rad**2-r**2)
    return s

tool_makers = [ ball_tool, endmill, vee_common(30), vee_common(45), vee_common(60), vee_common(90)]

def tool_info(tool_type=0, wdia=0, wdia2=-9999, degr=60, units=0):
    toolTypeStr = ("Ball End","Flat End","30 Degree","45 Degree","60 Degree","90 Degree","?")
    if units == 0:
        unitsStr = "inch"
    else:
        unitsStr = "mm"
    print("( Tool info: )")
    if wdia2 > wdia and degr > 0 and degr < 180:
        print("( Tool type: Cone {0} )".format(toolTypeStr[tool_type]))
        print("( Tool diameter: {0}{1} )".format(wdia2,unitsStr))
        print("( Tool tip diameter: {0}{1} )".format(wdia,unitsStr))
        print("( Tool angle: {0}degree )".format(degr))
        print("( Tool conical segment approx length: {0:.2f}{1} )".format(((wdia2/2)/tan(radians(degr/2)))-((wdia/2)/tan(radians(degr/2))),unitsStr))
    else:
        print("( Tool type: {0} )".format(toolTypeStr[tool_type]))
        print("( Tool diameter: {0}{1} )".format(wdia,unitsStr))