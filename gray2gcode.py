import datetime
from math import *
import numpy as np
import sys, os

from grayutil import Convert_Scan_Increasing, Convert_Scan_Decreasing, Convert_Scan_Alternating, Convert_Scan_Upmill, Convert_Scan_Downmill, Reduce_Scan_Lace, ArcEntryCut, correct_offset, make_tool_shape, optim_tool_shape, ball_tool, tool_info, make_tool_shape, tool_makers
from author import Gcode

unitcodes = ['G20', 'G21']
convert_makers = [ Convert_Scan_Increasing, Convert_Scan_Decreasing, Convert_Scan_Alternating, Convert_Scan_Upmill, Convert_Scan_Downmill ]

prn_detail = 0
epsilon = 1e-5
epsilon16 = 1e-16
is_ddd1 = True
roughing_depth_delta = 0.2 

importErrStr = ""

platformWin = True
if (os.name != "nt"):
  platformWin = False

try:
    from PIL import Image
except ImportError:
    importingError = True
    importErrStr += "Pillow\n"

try:
    import numpy as numarray
    import numpy.core
    olderr = numpy.core.seterr(divide='ignore')
    plus_inf = (np.array((1.,))/0.)[0]
    numpy.core.seterr(**olderr)
except ImportError:
    importingError = True
    importErrStr += "numpy\n"

class Converter:
    def __init__(self,
            image, units, tool_shape, pixelsize, pixelstep, safetyheight, \
            tolerance, feed, convert_rows, convert_cols, cols_first_flag,
            entry_cut, spindle_speed, roughing_offset, roughing_depth,
            roughing_feed,background_border,cut_top_jumper,
            optimize_path,layer_by_layer,
            max_bg_len,pattern_objectiv,
            roughing_minus_finishing,pixelstep_roughing,tool_roughing,min_delta_rmf,
            previous_offset):
        self.image = image
        self.units = units
        self.tool = tool_shape
        self.pixelsize = pixelsize
        self.pixelstep = pixelstep
        self.safetyheight = safetyheight
        self.tolerance = tolerance
        self.base_feed = feed
        self.convert_rows = convert_rows
        self.convert_cols = convert_cols
        self.cols_first_flag = cols_first_flag
        self.entry_cut = entry_cut
        self.spindle_speed = spindle_speed
        self.roughing_offset = roughing_offset
        self.previous_offset = previous_offset
        self.roughing_depth = roughing_depth
        self.roughing_feed = roughing_feed
        self.background_border = background_border
        self.cut_top_jumper = cut_top_jumper
        self.optimize_path = optimize_path
        self.layer_by_layer = layer_by_layer
        self.max_bg_len = max_bg_len
        self.pattern_objectiv = pattern_objectiv
        self.row_mill = (self.convert_rows != None)
        self.start_moment = datetime.datetime.now()

        self.roughing_offset = correct_offset(self.roughing_offset, self.pixelsize)
        self.previous_offset = correct_offset(self.previous_offset, self.pixelsize)

        self.layer = 0
        self.MaxBackground_down = (self.image.min()+self.background_border);
        self.MaxBackground_up = (self.image.max()-self.background_border);
        if prn_detail > 0 and self.background_border > .0:
            if self.cut_top_jumper:
                print("(Background border = {0} units, Background down = {1} units, Background up = {2} units)".format(self.background_border,\
                    self.MaxBackground_down,self.MaxBackground_up))
            else:
                print("(Background border = {0} units, Background down = {1} units)".format(self.background_border,self.MaxBackground_down))

        w, h = self.w, self.h = image.shape

        self.roughing_minus_finishing = roughing_minus_finishing
        if self.roughing_minus_finishing:
            self.pixelstep_roughing = pixelstep_roughing
            self.tool_roughing = tool_roughing
            self.min_delta_rmf = min_delta_rmf

            #I did not use the function 'get_RowCol' to quickly work out.
            if self.row_mill:
                self.map_tool2 = np.zeros((w, h), dtype=np.float32) - plus_inf

                # map_rmf>=0 - True;  map_rmf<0 - False
                self.map_rmf = np.zeros((w, h), dtype=np.float32) - plus_inf
            else:
                self.map_tool2 = np.zeros((h, w), dtype=np.float32) - plus_inf

                # map_rmf>=0 - True;  map_rmf<0 - False
                self.map_rmf = np.zeros((h, w), dtype=np.float32) - plus_inf
            self.map_tool2 = np.zeros((w, h), dtype=np.float32) - plus_inf

            # map_rmf>=0 - True;  map_rmf<0 - False
            #self.map_rmf = np.zeros((w, h), 'Float32') - plus_inf

        self.cache = {}

        self.cache_abs = {}

        ts = self.ts = tool_shape.shape[0]
        if prn_detail > 0: print("(Tool shape = {0} pixels)".format(ts))

        # "-1" - for functions get_dz_dy and get_dz_dx - code "+1"
        self.h1 = h - ts -1
        self.w1 = w - ts -1

        self.ts2 = self.ts/2

        #self.tool_shape = tool_shape * self.pixelsize * ts / 2;
    
    def one_pass(self):
        g = self.g
        g.set_feed(self.feed)

        self.layer = self.layer + 1
        if prn_detail > 0:
            print("(Layer {0} layer depth {1})".format(self.layer,self.layer_depth))

        if self.convert_cols and self.cols_first_flag:
            self.row_mill = False
            self.g.set_plane(19)
            if self.roughing_minus_finishing:
                if prn_detail > 0: print("(Error(!) This function is not implemented!)")

            elif self.pattern_objectiv:
                self.mill_objectiv(self.convert_cols, True)
            else:
                self.mill_cols(self.convert_cols, True)
            if self.convert_rows: g.safety()

        if self.convert_rows:
            self.row_mill = True
            self.g.set_plane(18)
            if self.pattern_objectiv:
                self.mill_objectiv(self.convert_rows, not self.cols_first_flag)
            else:
                self.mill_rows(self.convert_rows, not self.cols_first_flag)

        if self.convert_cols and not self.cols_first_flag:
            self.row_mill = False
            self.g.set_plane(19)
            if self.convert_rows: g.safety()
            if self.pattern_objectiv:
                self.mill_objectiv(self.convert_cols, not self.convert_rows)
            else:
                self.mill_cols(self.convert_cols, not self.convert_rows)

        if self.convert_cols:
            self.convert_cols.reset()
        if self.convert_rows:
            self.convert_rows.reset()
        g.safety()


    def get_rmf_map_tool_next(self,base_image,map_tool1,roughing_offset,tool_diameter,tool_type,pixelstep,roughing_depth,min_delta_rmf):
        jrange = range(self.w1)
        irange = range(self.h1)
        trange = range(0,self.ts)

        for y in jrange:    #lines image
            for x in irange:    #pixels image

                for i in trange:    #lines tool
                    for j in trange:    #pixels tool

                        ty = i+y
                        tx = j+x

                        map_tool1[ty,tx] = self.map_tool2[ty,tx]

        #clear caches
        self.map_rmf[:] = -plus_inf
        self.map_tool2[:] = -plus_inf
        self.cache.clear()
        self.cache_abs.clear()

        # new params and tool
        if self.roughing_offset != roughing_offset:
            self.roughing_offset = roughing_offset
            self.image = base_image
            self.image = self.make_offset(self.roughing_offset,self.image)

        self.tool = make_tool_shape(tool_type, tool_diameter, self.pixelsize)
        self.tool = optim_tool_shape(self.tool,-base_image.min(),self.roughing_offset,max(self.pixelsize,self.tolerance))

        self.pixelstep = pixelstep
        self.roughing_depth = roughing_depth
        self.min_delta_rmf = min_delta_rmf

        self.ts = self.tool.shape[0]
        if prn_detail > 0: print("(Next tool shape = {0} pixels)".format(self.ts))

        # "-1" - for functions get_dz_dy and get_dz_dx - code "+1"
        self.h1 = self.h - self.ts -1
        self.w1 = self.w - self.ts -1

        self.ts2 = self.ts/2
                        
        return map_tool1

    def make_offset(self,offset,image):
        if offset > epsilon16:
            rough = make_tool_shape(ball_tool,
                                2*offset, self.pixelsize, True)
            rough = optim_tool_shape(rough,-image.min(),offset,max(self.pixelsize,self.tolerance))
            w, h = image.shape
            tw, th = rough.shape
            w1 = w + tw
            h1 = h + th
            nim1 = np.zeros((w1, h1), dtype=np.float32) + image.min()

            nim1[int(tw/2):int(tw/2+w), int(th/2):int(th/2+h)] = image
            image = np.zeros((w,h), dtype=np.float32)
            for j in range(0, w):
                for i in range(0, h):
                    image[j,i] = (nim1[j:j+tw,i:i+th] - rough).max()
        return image

    def convert(self):
        self.g = g = Gcode(safetyheight=self.safetyheight,
                           tolerance=self.tolerance,
                           spindle_speed=self.spindle_speed,
                           units=self.units)
        g.begin()
        g.continuous(self.tolerance)
        g.safety()

        if self.safetyheight == 0 and prn_detail > -1: 
            print("(Warning: safety height = 0! You can give error like: 'Start of arc is the same as end!')")


        if prn_detail > -1 and self.MaxBackground_up <= self.MaxBackground_down:

                print("Error(!): Max background border down[{0}] >= Max background border up[{1}]! G-kode can not be formed.".format(self.MaxBackground_down, self.MaxBackground_up))
                print("     ...check background_border and image depth!")
                return


        self.layer_depth_prev = 9999.0    #'layer_depth_prev' need for 'optimize_path'
        if self.pattern_objectiv:

            base_image = self.image
            self.feed = self.roughing_feed

            if not self.roughing_minus_finishing:
                self.image = self.make_offset(self.roughing_offset,self.image)
            else:
                
                offset_image = self.make_offset(self.previous_offset,self.image)
                self.layer_depth = self.image.min()
                map_tool1 = self.get_rmf_map_tool(offset_image,self.tool_roughing,self.previous_offset,self.pixelstep_roughing)

                self.image = self.make_offset(self.roughing_offset,self.image)
                self.ro = self.roughing_offset
                self.set_rmf(base_image,map_tool1)

                #comented corth not used get_z in set_rmf and get_rmf_map_tool
                #self.cache.clear()





                #The following code is needed if you are using not two, but 3, 4, 5, ... cutter, 
                # and want to make a difference between:
                #   what is left clean after used cutter 
                #       and next cutter.

                '''#----------------------------------------------------------------
                # delta_map_tool2 = min(map_tool1,map_tool2)
                next_roughing_offset = 0.3
                next_tool_diameter = 1.5
                next_tool_type = ball_tool
                next_pixelstep = 1
                next_roughing_depth = 1.7
                next_min_delta_rmf = 2.0 #0.0 #1.8 #
                map_tool2 = self.get_rmf_map_tool_next(base_image,map_tool1,next_roughing_offset,next_tool_diameter,\
                                            next_tool_type,next_pixelstep,next_roughing_depth,next_min_delta_rmf)
                self.set_rmf(base_image,map_tool2)
                #----------------------------------------------------------------'''


                '''#----------------------------------------------------------------
                # delta_map_tool3 = min(map_tool2,map_tool3)
                next_roughing_offset = 0
                next_tool_diameter = 0.4
                next_tool_type = ball_tool
                next_pixelstep = 1
                next_roughing_depth = 0.4
                next_min_delta_rmf = 1.2 #0.3 #
                map_tool3 = self.get_rmf_map_tool_next(base_image,map_tool2,next_roughing_offset,next_tool_diameter,\
                                            next_tool_type,next_pixelstep,next_roughing_depth,next_min_delta_rmf)
                self.set_rmf(base_image,map_tool3)
                #----------------------------------------------------------------'''
            

        elif (self.roughing_depth > epsilon16 and self.roughing_offset > epsilon16) or (self.roughing_depth > epsilon16 and self.optimize_path > epsilon16):
            base_image = self.image
            self.image = self.make_offset(self.roughing_offset,self.image)

            self.feed = self.roughing_feed
            self.ro = self.roughing_offset
            m = self.image.min() + self.ro

            r = -self.roughing_depth
            self.layer_depth = .0

            while r > m:
                self.layer_depth = r
                self.one_pass()
                self.layer_depth_prev = self.layer_depth    #'layer_depth_prev' need for 'optimize_path'
                r = r - self.roughing_depth
            if self.layer_depth_prev > m + epsilon:
                self.layer_depth = m
                if prn_detail > 0: print("(Layer: {0} previous LD={1} < m[{2}]+epsilon[{3}]={4})".format(self.layer,self.layer_depth,m,epsilon,(m + epsilon)))
                self.one_pass()
                self.layer_depth_prev = self.layer_depth    #'layer_depth_prev' need for 'optimize_path'
            self.optimize_path = False
            self.image = base_image
            self.cache.clear()
        self.feed = self.base_feed
        if self.roughing_minus_finishing or self.optimize_path:
            self.ro = self.roughing_offset
        else:
            self.ro = 0.
        self.layer_depth = self.image.min()
        self.one_pass()
        g.end()
        timing = "End make at: "+str(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))+" Total time: "+str((datetime.datetime.now()-self.start_moment))
        if prn_detail > 0: print("( {0} )".format(timing))
        
    def get_dz_dy(self, x, y):
        y1 = max(0, y-1)
        y2 = min(self.image.shape[0]-1, y+1)
        #y2 = min(self.image.shape[0]-1-self.ts, y+1)
        dy = self.pixelsize * (y2-y1)
        return ((self.get_z(x, y2)) - (self.get_z(x, y1))) / dy
        
    def get_dz_dx(self, x, y):
        x1 = max(0, x-1)
        x2 = min(self.image.shape[1]-1, x+1)
        #x2 = min(self.image.shape[1]-1-self.ts, x+1)
        dx = self.pixelsize * (x2-x1)
        return ((self.get_z(x2, y)) - (self.get_z(x1, y))) / dx

    def get_z(self, x, y):

        try:
            return min(0, max(self.layer_depth, self.cache[x,y])) + self.ro
        except KeyError:
            m1 = self.image[y:y+self.ts, x:x+self.ts]
            #if you get ERROR in line below - this means 
            #   that Y or X LARGER than the size of image! Probably Y its X and X its Y... 
            self.cache[x,y] = d = (m1 - self.tool).max()
            return min(0, max(self.layer_depth, d)) + self.ro


    def get_RowCol(self, x, y):
        if self.row_mill:
            return x,y
        else:
            return y,x

    def get_z_abs(self, x, y):
        try:
            return min(0, self.cache_abs[x,y])
        except KeyError:
            m1 = self.image[y:y+self.ts, x:x+self.ts]
            #if you get ERROR in line below - this means 
            #   that Y or X LARGER than the size of image! Probably Y its X and X its Y... 
            d = (m1 - self.tool).max()
            self.cache_abs[x,y] = d
            return min(0,d)

    def get_rmf_map_tool(self,offset_image,tool_roughing,previous_offset,pixelstep_roughing):

        #Dictionary faster then array
        map_tool1 = {}  #np.zeros((self.w, self.h), 'Float32') - self.image.min()

        ts_roughing = tool_roughing.shape[0]
        if prn_detail > 0: print("(Previous tool shape: {0} pixels)".format(ts_roughing))

        if self.row_mill:
            jrange = range(0, self.w - ts_roughing - 1, pixelstep_roughing)
            irange = range(0, self.h - ts_roughing - 1)
            ln = self.w - ts_roughing - 1
        else:
            jrange = range(0, self.h - ts_roughing - 1, pixelstep_roughing)
            irange = range(0, self.w - ts_roughing - 1)
            ln = self.h - ts_roughing - 1

        if (len(jrange) == 0 or len(irange) == 0) and prn_detail > -1:
            print("(WARNING! Perhaps: previous tool diametr[{0} pixels] is larger then len of image[{1}, {2}]! )".format(ts_roughing,self.w,self.h))

            
        trange = range(0,ts_roughing)

        for ry in jrange:    #lines
            for rx in irange:    #pixels

                if self.row_mill:
                    y,x = ry,rx
                else:
                    x,y = ry,rx

                m1 = offset_image[y:y+ts_roughing, x:x+ts_roughing]
                hhh1 = (m1 - tool_roughing).max() + previous_offset

                for i in trange:    #lines tool
                    for j in trange:    #pixels tool

                        t = tool_roughing[i,j]
                        if isinf(t): continue

                        ty = i+y
                        tx = j+x
                        # self.image[ty,tx] <= hhh1 !!! t >= 0
                        dt = -self.image[ty,tx] + hhh1 + t
                        #dt = round(dt,16)

                        if dt < -epsilon and prn_detail > -1: print("( delta < -0.00001 ",ty,tx,dt,self.image[ty,tx], hhh1, t," )")
                        if dt < .0: dt = .0
                            
                        try:
                            if map_tool1[ty,tx] > dt: 
                                map_tool1[ty,tx] = dt
                        except KeyError:
                            map_tool1[ty,tx] = dt

        if prn_detail > 0: print("(End make map tool1. Map len: {0}. End at {1})".format(len(map_tool1),datetime.datetime.now()))
        if len(map_tool1) == 0 and prn_detail > -1: print("(WARNING! Map tool1 len: {0}! )".format(len(map_tool1)))

        return map_tool1

    def set_rmf(self,base_image,map_tool1):

        #map_tool2 = {}

        if self.row_mill:
            jrange = range(0, self.w1, self.pixelstep)
            irange = range(0, self.h1)
            ln = self.w1
        else:
            jrange = range(0, self.h1, self.pixelstep)
            irange = range(0, self.w1)
            ln = self.h1

        trange = range(0,self.ts)

        for lin in jrange:    #lines
            for pix in irange:    #pixels

                if self.row_mill:
                    y,x = lin,pix
                else:
                    x,y = lin,pix

                hhh1 = self.get_z(x,y)

                founded = False

                for i in trange:    #lines tool
                    for j in trange:    #pixels tool

                        t = self.tool[i,j]
                        if isinf(t): continue

                        ty = i+y
                        tx = j+x
                        #dt - without offset (base_image != self.image)!
                        im = -base_image[ty,tx]
                        dt = im + hhh1 + t      #-base_image[ty,tx] + min(0.0, hhh1 + t)
                        #dt = round(dt,16)
                        
                        if dt < -epsilon and prn_detail > -1: 
                            print("( delta[{0}] < -0.00001 y,x[{1}, {2}] image={3} hhh1={4} tool={5})".format(dt,ty,tx,base_image[ty,tx], hhh1, t))
                            if dt < .0: dt = .0
                        
                        if isinf(self.map_tool2[ty,tx]) or self.map_tool2[ty,tx] > dt: 
                            self.map_tool2[ty,tx] = dt
                        
                        try:
                            # "dt" must be >= "map_tool1", but not always
                            delta = map_tool1[ty,tx] - dt
                        except:
                            #1. Angles of image, at a round instrument.
                            #2. When 'previous tool diametr' is greater then image len.
                            #3. ?
                            delta = im
                            
                        try:
                            if delta >= self.min_delta_rmf:
                                #find minimum of delta
                                if isinf(self.map_rmf[lin,pix]) or delta >= self.map_rmf[lin,pix]:  #(y,x)!!!  Not (ty,tx)!!!
                                    #self.map_rmf[lin,pix] = delta
                                    if self.min_delta_rmf == 0 and delta == 0:
                                        if self.not_background(hhh1-self.roughing_offset,lin,pix):
                                            self.map_rmf[lin,pix] = epsilon16
                                        #else: self.map_rmf[lin,pix] = -1
                                    else:
                                        self.map_rmf[lin,pix] = delta

                            elif self.min_delta_rmf == 0 and delta < 0 and isinf(self.map_rmf[lin,pix]):
                                #self.map_rmf[lin,pix] = epsilon16

                                if self.not_background(hhh1-self.roughing_offset,lin,pix):
                                    self.map_rmf[lin,pix] = epsilon16
                                else:  
                                    self.map_rmf[lin,pix] = -1


                        except: pass

        if prn_detail > 0: 
            print("(Base min delta 'Roughing Minus Finish': {0} units)".format(self.min_delta_rmf))
            print("(     Min delta 'Roughing Minus Finish': {0} units)".format(self.map_rmf.min()))
            if self.map_rmf.min() < self.min_delta_rmf:
                print("(Warning: Min delta [{0}] < Base min delta[{1}] )".format(self.map_rmf.min(),self.min_delta_rmf))
            print("(     Max delta 'Roughing Minus Finish': {0} units)".format(self.map_rmf.max()))
            if self.map_rmf.max() < self.min_delta_rmf:
                print("(Error: Max delta [{0}] < Base min delta[{1}] )".format(self.map_rmf.max(),self.min_delta_rmf))

        if self.map_rmf.max() < 0 and prn_detail > -1: print("(Error() Min RMF: {0} < 0 units)".format(self.map_rmf.max()))
        if prn_detail > 0: print("(End make map rmf at {0})".format(datetime.datetime.now()))

        return



    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def pixel_use(self, hhh1,y,x):

        if self.roughing_minus_finishing:
            if (hhh1 + self.map_rmf[y,x]) < self.layer_depth:
                return False
            else:
                return True #Dont remove this line!

        return self.not_background(hhh1,y,x)

    def not_background(self, hhh1,y,x):

        if hhh1 >= self.MaxBackground_down \
            and (not self.cut_top_jumper or hhh1 <= self.MaxBackground_up):
            return True

        return False

    def processed_prev(self, hhh1):
        #not processed in previous layers - vertical
        if hhh1 <= self.layer_depth_prev:   #layer_depth_prev is (self.layer_depth+self.roughing_depth) but not alveys
        #if (hhh1-epsilon) ???  +self.ro ???
            return False
        return True

    def higher_then_layer_depth(self, hhh1):
        #higher then current layer depth - vertical
        # dont change '>' to '>=' ! Corse 'delineating' would not work in last layer...

        return hhh1 > self.layer_depth #if (hhh1-epsilon ???)  +self.ro?

    def not_processed_cur(self, y,x,processed_cur):
        #not processed in current layer - horizontal
        lin,pix = self.get_RowCol(y,x)
        try:
            d= processed_cur[y,x]   #777
            return False
        except KeyError:
            return True

    def gk_maker(self, make_gk, max_pix, processed_cur, convert_scan, primary,object_num,layer=-1):
        if layer == -1: clayer = self.layer  #from 1 - len
        else: clayer = layer                 #from 0 - (len-1) - self.roughing_minus_finishing
        d_len = 1
        lines = sorted(make_gk)
        ld = len(make_gk)

        convert_scan.reset()    #???

        if not self.row_mill: lines.reverse()
        for j in lines:
            Pixel_entry_f,Pixel_entry_l = make_gk[j]
                        
            #can't make g-code from 1 pixel
            P_f = max(0, Pixel_entry_f - d_len)
            P_l = min(max_pix, Pixel_entry_l + d_len+1)    #self.h1-1
            #if d_len == 0: P_l = min(self.h1,Pixel_entry_l + 1)

            if prn_detail > 1: 
                print("( GK> Layer: {0} obj: {1} LD[{2}] row: {3} [F{4}, L{5}] [{6}, {7}] )".format(clayer,object_num,\
                            self.layer_depth,j,Pixel_entry_f,Pixel_entry_l,P_f,P_l))


            # clean covered
            for i in range(Pixel_entry_f,Pixel_entry_l+1): processed_cur[j,i] = 7

            scan = []
            #I did not use the function 'get_RowCol' to quickly work out.
            if self.row_mill:
                #Rows
                y = (self.w1-j+self.ts2) * self.pixelsize
                for i in range(P_f,P_l):

                    x = (i+self.ts2) * self.pixelsize
                    milldata = (i, (x, y, self.get_z(i, j)),
                        self.get_dz_dx(i, j), self.get_dz_dy(i, j))
                    scan.append(milldata)

                # make g-code
                if len(scan) != 0:
                    for flag, points in convert_scan(primary, scan):
                        if flag:
                            self.entry_cut(self, points[0][0], j, points)   #ArcEntryCut
                        for p in points:
                            self.g.cut(*p[1])
                else:
                    if prn_detail > -1: print("Error(0) g-kode not make L: {0} [{1}, {2}] !!!".format(clayer,P_f,P_l))
            else:
                #Cols
                x = (j+self.ts2) * self.pixelsize
                scan = []
                for i in range(P_f,P_l):
                    y = (self.w1-i+self.ts2) * self.pixelsize
                    milldata = (i, (x, y, self.get_z(j, i)),
                      self.get_dz_dy(j, i), self.get_dz_dx(j, i))
                    scan.append(milldata)

                # make g-code
                if len(scan) != 0:
                    for flag, points in convert_scan(primary, scan):
                        if flag:
                            self.entry_cut(self, j, points[0][0], points)
                        for p in points:
                            self.g.cut(*p[1])
                else:
                    if prn_detail > -1: print("( ",clayer,", Error len(scan) == 0! g-kode not make from ",P_f," to ",P_l," !!!)")
            self.g.flush()
            
        self.g.safety()
        convert_scan.reset()

        if prn_detail > 1: print("(End of object: g.safety, convert_rows.reset )")

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def get_entry_pixels_go_end(self, start, stop, j, processed_cur,max_pix):
        #This procedure is very sensitive to changes in! Very, very, very sensitive...
        # so if you decided to change something here quickly, without understanding how it works - then you probably something to break...
        #   Be ready to rewrite these procedures from scratch.
        # Procedures that are related to a single logic: get_dict_gk, get_entry_pixels_go_end, get_entry_pixels_go_start, higher_then_layer_depth
        #   if you make changes in one procedure, then make changes to other procedures.
        Pixel_entry_f = -1
        Pixel_entry_l = -1
        Blen = 0;

        y,x = self.get_RowCol(j,start)
        hhh1 = self.get_z_abs(x,y)
        Curn_U = self.pixel_use(hhh1,j,start)
        Curn_D = self.higher_then_layer_depth(hhh1)
        hhh2 = hhh1


        First_border = False
        Last_border  = False
        verify_prev = True  #not entry first/last
        verify_curn = True  #not entry last /right

        #-----------------------------------------------------------------------------------------------
        #delineating description:
        # delineate are areas in contact picture with the background.
        # delineate is are areas in which the g-code is generated, no matter pixel Used - or not!
        # without "delineate" and with "background_border > 0" - the g-code will be with holes...
        #
        # for delineate - compared paremetry two adjacent points
        # for delineating - one of the pixels - mast be used, 
        #  and they must lie in different layers, separated by 'layer_depth'.
        # 
        #
        # Prev_U Curn_U Next_U - pixel Used or not.
        # Prev_D Curn_D Next_D - pixel higher_then_layer_depth or not
        # 
        # verify_prev - if False - then delineate and previous pixel is Used. If True - not delineate.
        # verify_curn - if False - then delineate and current  pixel is Used. If True - not delineate.
        #
        #-delineating description
        #-----------------------------------------------------------------------------------------------


        #delineating - 'start' pixel:
        t_i = start-1   #prev pixel
        y,x = self.get_RowCol(j,t_i)
        try:
            hhh3 = self.get_z_abs(x,y)
            Prev_U = self.pixel_use(hhh3,j,t_i)
            Prev_D  = self.higher_then_layer_depth(hhh3)
        except:
            hhh3 = -777
            Prev_U = False 
            Prev_D = True

        #one of the pixels - mast be used, 
        # and they must lie in different layers, separated by 'layer_depth'.
        if Curn_D != Prev_D and ((Curn_U and Curn_D) or (Prev_U and Prev_D)):
            if Prev_D: 
                if prn_detail > 9: print("( ??? [e.0] first border ",Curn_D,Prev_D,Curn_U,Prev_U,t_i,j,hhh3," )")
                verify_prev = False
                Last_border = True
            elif Curn_D: 
                if prn_detail > 9: print("(  [e.0] last border ",Curn_D,Prev_D,Curn_U,Prev_U,start,j,hhh1,self.layer_depth,(hhh1 <= self.layer_depth),self.higher_then_layer_depth(hhh1)," )")
                verify_curn = False
                First_border = True
        #-delineating  - 'start' pixel
        
        for i in range(start,max_pix+1):    #pixels
            next_i = i+1


            y,x = self.get_RowCol(j,next_i)
            try:
                hhh2 = self.get_z_abs(x,y)
                Next_U = self.pixel_use(hhh2,j,next_i)
                Next_D = self.higher_then_layer_depth(hhh2)
            except:
                Next_U = False
                Next_D = True

            #self.delineating: 
            #one of the pixels - mast be used, 
            # and they must lie in different layers, separated by 'layer_depth'.
            if Curn_D != Next_D and ((Curn_U and Curn_D) or (Next_U and Next_D)): 
                if Next_D: 
                    verify_prev = False
                    First_border = True
                    if prn_detail > 9: print("(  [e] first border ",Curn_D,Next_D,Curn_U,Next_U,next_i,j,hhh2," )")
                else: 
                    verify_prev = True
                    verify_curn = False
                    Last_border = True
                    if prn_detail > 9: print("(  [e] last border ",Curn_D,Next_D,Curn_U,Next_U,i,j,hhh1," )")
            else: verify_prev = True
            #-delineating

            Cur_tmp = False
            if Curn_U and self.not_processed_cur(j,i,processed_cur):

                if self.optimize_path and verify_curn and self.processed_prev(hhh1): 
                    if Pixel_entry_f != -1:
                        break
                else:
                    Cur_tmp = Curn_U    #==True



            if Cur_tmp:
                Pixel_entry_l = i

                #remember first
                if Pixel_entry_f == -1: Pixel_entry_f = i

                Blen = 0
            else:
                if i > stop:
                    # when roughing_minus_finishing and "delta" - is smol - g-kod maked not in all layers!
                    #elif i > stop or (self.roughing_minus_finishing and start != 0): Blen = Blen + 1
                    if Pixel_entry_f != -1: Blen = Blen + 1
                    else: break
                else:
                    if Pixel_entry_f != -1: Blen = Blen + 1

                if Blen > self.max_bg_len: 
                    break

            hhh1 = hhh2
            Curn_D = Next_D
            Curn_U = Next_U
            verify_curn = verify_prev
        return Pixel_entry_f, Pixel_entry_l, First_border, Last_border

    def get_entry_pixels_go_start(self, start, stop, j, processed_cur,min_pix):
        #This procedure is very sensitive to changes in! Very, very, very sensitive...
        # so if you decided to change something here quickly, without understanding how it works - then you probably something to break...
        #   Be ready to rewrite these procedures from scratch.
        # Procedures that are related to a single logic: get_dict_gk, get_entry_pixels_go_end, get_entry_pixels_go_start, higher_then_layer_depth
        #   if you make changes in one procedure, then make changes to other procedures.
        Pixel_entry_f = -1
        Pixel_entry_l = -1
        Blen = 0;

        y,x = self.get_RowCol(j,start)
        hhh1 = self.get_z_abs(x,y)
        Curn_U = self.pixel_use(hhh1,j,start)
        Curn_D = self.higher_then_layer_depth(hhh1)
        hhh2 = hhh1
 
        First_border = False
        Last_border = False
        verify_prev = True
        verify_curn = True



        #-----------------------------------------------------------------------------------------------
        #delineating description:
        # delineate are areas in contact picture with the background.
        # delineate is are areas in which the g-code is generated, no matter pixel Used - or not!
        # without "delineate" and with "background_border > 0" - the g-code will be with holes...
        #
        # for delineate - compared paremetry two adjacent points
        # for delineating - one of the pixels - mast be used, 
        #  and they must lie in different layers, separated by 'layer_depth'.
        # 
        #
        # Prev_U Curn_U Next_U - pixel Used or not.
        # Prev_D Curn_D Next_D - pixel higher_then_layer_depth or not
        # 
        # verify_prev - if False - then delineate and previous pixel is Used. If True - not delineate.
        # verify_curn - if False - then delineate and current  pixel is Used. If True - not delineate.
        #
        #-delineating description
        #-----------------------------------------------------------------------------------------------

        #self.delineating - 'start' pixel:
        t_i = start+1   #prev pixel

        y,x = self.get_RowCol(j,t_i)
        try:
            hhh3 = self.get_z_abs(x,y)
            Prev_U = self.pixel_use(hhh3,j,t_i)
            Prev_D  = self.higher_then_layer_depth(hhh3)
        except:
            hhh3 = -777
            Prev_U = False
            Prev_D = True


        #one of the pixels - mast be used, 
        # and they must lie in different layers, separated by 'layer_depth'.
        if Curn_D != Prev_D and ((Curn_U and Curn_D) or (Prev_U and Prev_D)): 
            if Prev_D: 
                verify_prev = False
                First_border = True
                if prn_detail > 9: print("( ??? [l.0] first border ",Curn_D,Prev_D,Curn_U,Prev_U,t_i,j,hhh3," )")
            elif Curn_D: 
                verify_curn = False
                Last_border = True
                if prn_detail > 9: print("(  [l.0] last border ",Curn_D,Prev_D,Curn_U,Prev_U,t_i,j,hhh1," )")
        #-delineating - 'start' pixel

        for i in range(start, min_pix-1, -1):   #pixels - go to the left

            next_i = i-1
            if next_i >= 0:
                y,x = self.get_RowCol(j,next_i)
                hhh2 = self.get_z_abs(x,y)
                Next_U = self.pixel_use(hhh2,j,next_i)
                Next_D = self.higher_then_layer_depth(hhh2)
            else:
                Next_U = False
                Next_D = True

            #self.delineating: 
            #one of the pixels - mast be used, 
            # and they must lie in different layers, separated by 'layer_depth'.
            if Curn_D != Next_D and ((Curn_U and Curn_D) or (Next_U and Next_D)):
                if Next_D: 
                    verify_prev = False
                    Last_border = True
                    if prn_detail > 9: print("(  [l] first border ",Curn_D,Next_D,Curn_U,Next_U,i-1,j,hhh2," )")
                else: 
                    verify_prev = True
                    verify_curn = False
                    First_border = True
                    if prn_detail > 9: print("(  [l] last border ",Curn_D,Next_D,Curn_U,Next_U,i,j,hhh1," )")
            else: verify_prev = True
            #-delineating

            Cur_tmp = False
            if Curn_U and self.not_processed_cur(j,i,processed_cur):

                if self.optimize_path and verify_curn and self.processed_prev(hhh1): 
                    if Pixel_entry_l != -1:
                        break
                else:
                    Cur_tmp = Curn_U    #==True


            if Cur_tmp:
                #find first
                Pixel_entry_f = i

                #remember last
                if Pixel_entry_l == -1: Pixel_entry_l = i

                Blen = 0
            else:
                if i < stop:
                    # when roughing_minus_finishing and "delta" is small - g-kod ne na vseh sloyah formiruetsa
                    #elif i < stop or (self.roughing_minus_finishing and start != 0): Blen = Blen + 1
                    if Pixel_entry_l != -1: Blen = Blen + 1 #countdown
                    else: break
                else:
                    if Pixel_entry_l != -1: Blen = Blen + 1 #countdown

                if Blen > self.max_bg_len: 
                    break
            hhh1 = hhh2
            Curn_D = Next_D
            Curn_U = Next_U
            verify_curn = verify_prev
        return Pixel_entry_f, Pixel_entry_l, First_border, Last_border

    def get_dict_gk(self,processed_cur,line_cache,object_num,mas_obj_filter):
        #This procedure is very sensitive to changes in!
        # so if you decided to change something here quickly, without understanding how it works - then you probably something to break...
        #   Be ready to rewrite these procedures from scratch.
        #       if you decided to change something - cerful - this proc can entered into an infinite loop ...
        # Procedures that are related to a single logic: get_dict_gk, get_entry_pixels_go_end, get_entry_pixels_go_start, higher_then_layer_depth
        #   if you make changes in one procedure, then make changes to other procedures.
        make_gk = {}
        mas_p = (-1,-1)
        object_is_found = False

        Pixel_entry_f = -1
        Pixel_entry_l = -1
        Pixel_entry_f_prev = -1
        Pixel_entry_l_prev = -1
        Pixel_entry_f_next = -1
        Pixel_entry_l_next = -1
        First_border = False
        Last_border = False
        ld = len(mas_obj_filter)
        gk_rows = mas_obj_filter.keys()
        gk_rows.sort()  #this importent! Don't remove!

        Prev_line = -1
        st = 1
        if not self.row_mill: gk_rows.reverse()

        for j in gk_rows:   #lines
            if self.row_mill:
                if line_cache > j: continue
            else:
                if line_cache < j: continue

            min_pix,max_pix = mas_obj_filter[j]

            #if j >= 50 and object_num > 3: return

            if Pixel_entry_f == -1:
                # find Pixel entry
                #*************************************************************************************************************************************************
                Pixel_entry_f,Pixel_entry_l,First_border,Last_border = self.get_entry_pixels_go_end(min_pix, max_pix, j, processed_cur,max_pix)
                #*************************************************************************************************************************************************

                if not object_is_found:
                    if self.row_mill:
                        if line_cache < j: line_cache = j
                    else:
                        if line_cache > j: line_cache = j

                if prn_detail > 3: print("( f-d entry rezult: L{0} obj{1} lin{2} PL{3} border[f{4}, l{5}] lim[{6}, {7}] - next[{8},{9}])".format(\
                                    self.layer,object_num,j,Prev_line,First_border,Last_border,min_pix,max_pix,Pixel_entry_f,Pixel_entry_l) )
            else:

                Pixel_entry_f_next = -1
                Pixel_entry_l_next = -1

                if (Pixel_entry_f > max_pix) or (min_pix > Pixel_entry_l):
                    #end of object
                    pass
                elif (st % 2) and First_border:
                    # fist - go left from Pixel_entry_f - then - right
                    if prn_detail > 3: print("( f-d from First border: L{0} obj{1} lin{2} PL{3} border[f{4}, l{5}] lim[{6}, {7}] from[{8}, {9}])".format(\
                                        self.layer,object_num,j,Prev_line,First_border,Last_border,min_pix,max_pix,Pixel_entry_f,Pixel_entry_l)  )  
                    
                    CurF = max(Pixel_entry_f,min_pix)
                    CurL = min(Pixel_entry_l,max_pix)

                    # left ****************************************************************************************************************************************
                    Pixel_entry_f_next,Pixel_entry_l_next, First_border, Last_border = self.get_entry_pixels_go_start(CurF,CurF , j, processed_cur,min_pix)
                    #*******************************************************************************************************************************************


                    # go to the left  from Pixel_entry_f
                    # right **************************************************************************************************************************
                    if Pixel_entry_l_next == -1:
                        start_from = CurF
                        if prn_detail > 4: print("( -Wasn't f-d: border[f{0}, l{1}] Next[F{2}, L{3}] start[{4}])".format(\
                                            First_border,Last_border, Pixel_entry_f_next, Pixel_entry_l_next,start_from))
                    else:
                        start_from = Pixel_entry_l_next
                        if prn_detail > 4: print("( + Found: border[f{0}, l{1}] Next[F{2}, L{3}] start[{4}])".format(\
                                            First_border,Last_border, Pixel_entry_f_next, Pixel_entry_l_next,start_from))


                    tPixel_entry_f,Pixel_entry_l_next,tFirst_border,tLast_border = self.get_entry_pixels_go_end(\
                        start_from, CurL, j, processed_cur,max_pix)

                    if Pixel_entry_f_next == -1: Pixel_entry_f_next = tPixel_entry_f

                    First_border = (tFirst_border or First_border)
                    Last_border  = (tLast_border or Last_border)

                    if prn_detail > 2: print("(f-d from First border rezult: L{0} obj{1} lin{2} border[f{3}, l{4}] Next[{5}, {6}])".format(\
                                        self.layer,object_num,j,First_border,Last_border, Pixel_entry_f_next, Pixel_entry_l_next))
                    #**************************************************************************************************************************
                else:
                    # fist - go right - then - left
                    if prn_detail > 3: print("( f-d from Last border: L{0} obj{1} lin{2} PL{3} border[f{4}, l{5}] lim[{6}, {7}] from[{8}, {9}])".\
                                        format(self.layer,object_num,j,Prev_line,First_border,Last_border,min_pix,max_pix,Pixel_entry_f,Pixel_entry_l)    )
                    
                    CurF = max(Pixel_entry_f,min_pix)
                    CurL = min(Pixel_entry_l,max_pix)

                    #**************************************************************************************************************************
                    Pixel_entry_f_next,Pixel_entry_l_next,First_border,Last_border = self.get_entry_pixels_go_end(CurL, CurL, j,\
                                                                                                         processed_cur,max_pix)
                    #**************************************************************************************************************************

                    # go to the left  from Pixel_entry_l
                    #**************************************************************************************************************************
                    if Pixel_entry_f_next == -1:
                        start_from = CurL
                        if prn_detail > 4: print("( -Wasn't f-d: border[f{0}, l{1}] Next[F{2}, L{3}] start[{4}])".format(\
                                            First_border,Last_border, Pixel_entry_f_next, Pixel_entry_l_next,start_from))
                    else:
                        start_from = Pixel_entry_f_next
                        if prn_detail > 4: print("( + Found: border[f{0}, l{1}] Next[F{2}, L{3}] start[{4}])".format(\
                                            First_border,Last_border, Pixel_entry_f_next, Pixel_entry_l_next,start_from))


                    Pixel_entry_f_next, tPixel_entry_l, tFirst_border, tLast_border = self.get_entry_pixels_go_start(\
                        start_from,CurF, j, processed_cur,min_pix)

                    if Pixel_entry_l_next == -1: Pixel_entry_l_next = tPixel_entry_l

                    First_border = (tFirst_border or First_border)
                    Last_border  = (tLast_border or Last_border)

                    if prn_detail > 2: print("(f-d from Last border rezult: L{0} obj{1} lin{2} border[f{3}, l{4}] Next[{5}, {6}])".format(\
                                            self.layer,object_num,j, First_border,Last_border, Pixel_entry_f_next, Pixel_entry_l_next))
                    #**************************************************************************************************************************

                #if (Pixel_entry_f_next == -1 or Pixel_entry_l_next == -1) and (Pixel_entry_f_next != -1 or Pixel_entry_l_next != -1):
                #    print("Error() L{0} obj{1} lin{2} Next[{3}, {4}]".format(self.layer,object_num,j, Pixel_entry_f_next, Pixel_entry_l_next)


                Pixel_entry_f_next,Pixel_entry_l_next = self.chek_gr(Pixel_entry_f_next,Pixel_entry_l_next,min_pix,max_pix,object_num,j)

                #dont_cut_angles:
                if prn_detail > 8: 
                    print("( dont_cut_angles r:{0} F prev[{3}] current[{1}] next[{2}])".format(Prev_line,Pixel_entry_f,Pixel_entry_f_next,Pixel_entry_f_prev))
                    print("( dont_cut_angles r:{0} L prev[{3}] current[{1}] next[{2}])".format(Prev_line,Pixel_entry_l,Pixel_entry_l_next,Pixel_entry_l_prev))
                if st % 2:
                    if Pixel_entry_l_prev  > Pixel_entry_l: 
                        if prn_detail > 2: print("( dont_cut_angles r:{0} l_prev:{1} -> l:{2})".format(Prev_line,Pixel_entry_l_prev,Pixel_entry_l))
                        Pixel_entry_l = Pixel_entry_l_prev
                    if Pixel_entry_f_next != -1 and Pixel_entry_f_next < Pixel_entry_f: 
                        if prn_detail > 2: print("( dont_cut_angles r:{0} f_next:{1} -> f:{2})".format(Prev_line,Pixel_entry_f_next,Pixel_entry_f))
                        Pixel_entry_f = Pixel_entry_f_next
                else:
                    if Pixel_entry_l_next  > Pixel_entry_l: 
                        if prn_detail > 2: print("( dont_cut_angles r:{0} l_next:{1} -> l:{2})".format(Prev_line,Pixel_entry_l_next,Pixel_entry_l))
                        Pixel_entry_l = Pixel_entry_l_next
                    if Pixel_entry_f_prev !=-1 and Pixel_entry_f_prev < Pixel_entry_f: 
                        if prn_detail > 2: print("( dont_cut_angles r:{0} f_prev:{1} -> f:{2})".format(Prev_line,Pixel_entry_f_prev,Pixel_entry_f))
                        Pixel_entry_f = Pixel_entry_f_prev
                #-dont_cut_angles

                # make g-code from previous entrys
                if Pixel_entry_f != -1 and Pixel_entry_l != -1:
                    object_is_found = True
                    
                    if st == 1: mas_p = (Prev_line,Pixel_entry_l)   #point of entry - always Last
                    st = st + 1

                    make_gk[Prev_line] = (Pixel_entry_f,Pixel_entry_l)

                    if prn_detail > 1: 
                        print("( Add to dict GK: L{0} obj{1} lin{2} [F{3}, L{4}] st={5})".format(self.layer,object_num,\
                            Prev_line,Pixel_entry_f,Pixel_entry_l,(st-1)%2))   #,P_f,P_l,self.ts) 
                    if prn_detail > 2: print(" ")

                elif Pixel_entry_f != -1 or Pixel_entry_l != -1: 
                    if prn_detail > -1: print("( ERROR (*)******************",self.layer," lin: ",j, Pixel_entry_f, Pixel_entry_l," )")

                Pixel_entry_f_prev = Pixel_entry_f
                Pixel_entry_l_prev = Pixel_entry_l
                Pixel_entry_f = Pixel_entry_f_next
                Pixel_entry_l = Pixel_entry_l_next

                if object_is_found and Pixel_entry_f == -1 and Pixel_entry_l == -1: 
                    if prn_detail > 1: print("( Don't find Next[F{3}, L{4}] in lin:{2})".format(self.layer,object_num,j, Pixel_entry_f,Pixel_entry_l))
                    break

            Pixel_entry_f,Pixel_entry_l = self.chek_gr(Pixel_entry_f,Pixel_entry_l,min_pix,max_pix,object_num,j)
            Prev_line = j

        # make g-code from last entrys for last lin.
        if Pixel_entry_f != -1 and Pixel_entry_l != -1:
            object_is_found = True
            
            #dont_cut_angles:
            if st % 2:
                if Pixel_entry_l_prev  > Pixel_entry_l: 
                    if prn_detail > 2: print("( dont_cut_angles r:{0} l_prev:{1} -> l:{2})".format(j,Pixel_entry_l_prev,Pixel_entry_l))
                    Pixel_entry_l = Pixel_entry_l_prev
            else:
                if Pixel_entry_f_prev !=-1 and Pixel_entry_f_prev < Pixel_entry_f: 
                    if prn_detail > 2: print("( dont_cut_angles r:{0} f_prev:{1} -> f:{2})".format(j,Pixel_entry_f_prev,Pixel_entry_f))
                    Pixel_entry_f = Pixel_entry_f_prev
            #-dont_cut_angles

            if st == 1: mas_p = (j,Pixel_entry_l)   #point of entry - always Last

            make_gk[j] = (Pixel_entry_f,Pixel_entry_l)

            if prn_detail > 1: print("( Add to dict GK_: L{0} obj{1} lin{2} [F{3}, L{4}] st={5})".format(\
                                self.layer,object_num,j,Pixel_entry_f,Pixel_entry_l,st%2))   #,P_f,P_l,self.ts) 

        elif Pixel_entry_f != -1 or Pixel_entry_l != -1: 
            if prn_detail > -1: print("( ERROR (**)*****************",self.layer," lin: ",j, Pixel_entry_f, Pixel_entry_l," )")

        if prn_detail > 1:
            if object_is_found: 
                print("(End of obj: {0} lines: {1} entry point [{2}, {3}])".format(object_num,len(make_gk),mas_p[0],mas_p[1]) )
            else:
                print("(Object not found.)")

        return make_gk,line_cache,object_is_found,mas_p

    def chek_gr(self,first,last,min_pix,max_pix,object_num,line):
        if (first == -1) and (min_pix == -1):
            return first,last
        elif (first > max_pix) or (min_pix > last):
            #end of object
            first = -1
            last  = -1
            return first,last

        if first > max_pix:
            #suda ne doljno zaiti!!!!
            if prn_detail > -1: print("Error logik(!) Pixel_entry_f_next > max_pix: {0} > {1} layer: {2} obj: {3} line {4}".format(first, max_pix,self.layer,object_num,line))
            first = max_pix
        elif first < min_pix:
            #suda ne doljno zaiti!!!!
            if prn_detail > -1: print("Error logik(!) Pixel_entry_f_next < min_pix: {0} < {1} layer: {2} obj: {3} line {4}".format(first, min_pix,self.layer,object_num,line))
            first = min_pix

        if max_pix < last:
            #suda ne doljno zaiti!!!!
            if prn_detail > -1: print("Error logik(!) Pixel_entry_l_next > max_pix: {0} > {1} layer: {2} obj: {3} line {4}".format(last, max_pix,self.layer,object_num,line))
            last = max_pix
        elif min_pix > last:
            #suda ne doljno zaiti!!!!
            if prn_detail > -1: print("Error logik(!) Pixel_entry_l_next > min_pix: {0} < {1} layer: {2} obj: {3} line {4}".format(last, min_pix,self.layer,object_num,line))
            last = min_pix

        return first,last

    def find_next_e(self, mas_p,y1,x1,max_y,max_x):
        max_ = max(max_y,max_x)

        i = 0
        if prn_detail > 1: print("(start the search next entry from [",y1,x1,"] )")

        while True:

            if i > max_:
                if prn_detail > -1: print("( BREAK: LOGIK ERROR(0): ",i," )")
                break

            ww2 = {}
            i = i + 1

            #  _____
            # |     |
            # |  *  |
            # |_____|
            xrang_r  = range(max(x1-i  ,0),     min(x1+i+1,max_x+1)     )
            xrang_l  = range(min(x1+i,max_x),   max(x1-i-1,-1),       -1)
            yrang_up = range(max(y1-i+1,0),     min(y1+i  ,max_y+1)     )
            yrang_dn = range(min(y1+i-1,max_y), max(y1-i  ,-1),       -1)

            num = 0
            if y1-i >= 0:
                #if y1 == 23 and x1 == 0: print("1-",y1-i,xrang_r)
                for m in xrang_r:
                    num = num + 1
                    ww2[num] = (y1-i,m)
            if x1+i <= max_x:
                #if y1 == 23 and x1 == 0: print("2-",x1+i,yrang_up)
                for m in yrang_up:
                    num = num + 1
                    ww2[num] = (m,x1+i)
            if y1+i <= max_y:
                #if y1 == 23 and x1 == 0: print("3-",y1+i,xrang_l)
                for m in xrang_l:
                    num = num + 1
                    ww2[num] = (y1+i,m)
            if x1-i >= 0:
                #if y1 == 23 and x1 == 0: print("4-",x1-i,yrang_dn)
                for m in yrang_dn:
                    num = num + 1
                    ww2[num] = (m,x1-i)


            if len(ww2) == 0:
                if prn_detail > -1: 
                    print("( BREEEEEAK: LOGIK ERROR(!): ",i,len(mas_p),max_x,max_y,y1-i,y1+i,x1-i,x1+i," )")
                    print(mas_p)
                    #print(ww2)                
                break

            # cheking
            tmm = ww2.keys()
            tmm.sort()
            for num in tmm:
                y,x = ww2[num]
                try:
                    m = mas_p[y,x]
                    if prn_detail > 2: print("( find next entry in [",y,x,"] obj: ",m," iter: ",i," )")
                    return y,x
                except KeyError:
                    m = 0

        return y1,x1
        

        


    def BinSearchVirt(self,li, x):
        i = 0
        j = len(li)-1
        while i < j:
            m = int((i+j)/2)
            if x > li[m]:
                i = m+1
            else:
                j = m
        #here it does not matter j or i
        if li[j] == x:
            return True
        else:
            return False

    def objects_relate_to(self,obj_rt,rw,first, last):
        mas_rows = obj_rt.keys()   #[59, 60, 61]
        mas_rows.sort()
        if self.BinSearchVirt(mas_rows, rw):
            Cur_f, Cur_l = obj_rt[rw]
            if (first <= Cur_f <= last) or (Cur_f <= first <= Cur_l):
                return True
        return False


    def get_mas_glp(self,objects_layer,layer,object_num_prt):

        #warning: objects on one layer may overlap ONLY due corrections angles 'dont_cut_angles' in 'get_dict_gk'
        #Warning: objects may intersect at several locations simultaneously

        if prn_detail > 1: print("(Create a list of adjacent objects of this layer: {0} parent object num: {1})".format(layer,object_num_prt))

        max_line,max_pix = self.get_RowCol(self.w, self.h)  # not (self.? - self.ts_roughing - 1) !?

        mas_glp = {}
        ld = len(objects_layer)
        if ld <= 1: return mas_glp

        for num in objects_layer: #main obj
            y,x,make_gk = objects_layer[num]      #objects_layer - {1: {59: (0, 0), 60: (0, 3), 61: (0, 3)}}
            for rw in make_gk:  #rows
                Cur_f, Cur_l = make_gk[rw]

                for num2 in objects_layer:    #obj relate_to
                    if num2 == num: continue
                    y,x,make_gk2 = objects_layer[num2]
                    if rw - 1 >= 0 \
                        and self.objects_relate_to(make_gk2,rw-1,Cur_f, Cur_l):
                        if prn_detail > 1: print("( gluing pieces N {0} and N {1} lines: {2},{3} pixels: {4},{5})".format(num,num2,rw,rw-1,Cur_f, Cur_l))
                        try:
                            if num2 not in mas_glp[num]: mas_glp[num].append(num2)
                        except:
                            try:
                                mas_glp[num].append(num2)
                            except:
                                mas_glp[num] = [num2]
                    elif rw + 1 <= max_line \
                        and self.objects_relate_to(make_gk2,rw+1,Cur_f, Cur_l):
                        if prn_detail > 1: print("( gluing pieces N {0} and N {1} lines: {2},{3} pixels: {4},{5})".format(num,num2,rw,rw+1,Cur_f, Cur_l))
                        try:
                            if num2 not in mas_glp[num]: mas_glp[num].append(num2)
                        except:
                            try:
                                mas_glp[num].append(num2)
                            except:
                                mas_glp[num] = [num2]
                    #??? elif self.objects_relate_to(make_gk2,rw,Cur_f, Cur_l):

        #print("mas_glp: ",mas_glp)
        return mas_glp

    def get_mas_parents(self,prt_gk,mas_pieces,layer,num):

        mas_parents = []
        objects_layer = mas_pieces[layer-1]

        for rw in prt_gk:  #rows of parent object
            Cur_f, Cur_l = prt_gk[rw]

            #objects_layer - [(1, 59, 0, {59: (0, 0), 60: (0, 3), 61: (0, 3)})]
            for num2 in objects_layer:    #parents objects in (layer-1)
                y,x,make_gk = objects_layer[num2]
                if num2 in mas_parents: continue
                if self.objects_relate_to(make_gk,rw,Cur_f, Cur_l):
                    if prn_detail > 2: 
                        print("( child pieces N {1} in L: {0} has parent N {2} in L {6} line: {3} pixels: {4},{5})".format(\
                                    layer,num,num2,rw,Cur_f, Cur_l,layer-1))
                    mas_parents.append(num2)

        #print("mas_parents: ",mas_parents)
        return mas_parents

    def get_tree_prt(self,mas_pieces):

        #: overlapping objects on different layers are considered related
        #: child object is an object on the layer that is deeper
        #: child may have more than one parent objects
        #: parent object can have multiple children

        if prn_detail > 0: print("(Create a tree of relationships between objects. Start at {0})".format(datetime.datetime.now()))

        max_line,max_pix = self.get_RowCol(self.w, self.h)  # not (self.? - self.ts_roughing - 1) !?

        tree_prt = {}   #{layer0={num child0_in_layer1=[num parent1,num parent2,num parent3,...]}, {num child2_in_layer0=[...]},...}

        ld = len(mas_pieces)
        if ld <= 1: return tree_prt

        #print(radical parents
        if prn_detail > 2: 
            for object_num in mas_pieces[0]:
                y,x,make_gk = mas_pieces[0][object_num]
                print("( L: 0 radical parent N {0} enter point[{1},{2}])".format(object_num,y,x))

        lrange = range(1,ld)
        for layer in lrange:
            parents_of_object = {}
            for object_num in mas_pieces[layer]:
                y,x,make_gk = mas_pieces[layer][object_num]
                #objects_intersect
                mas_parents = self.get_mas_parents(make_gk,mas_pieces,layer,object_num)
                if len(mas_parents) > 0:
                    parents_of_object[object_num] = mas_parents

            tree_prt[layer] = parents_of_object
        #print("tree_prt: ",tree_prt)
        return tree_prt

    def mill_objectiv(self, convert_scan, primary):

        if prn_detail > 0: 
            if self.roughing_minus_finishing:
                print("(Start mill objectiv mode. Previous cutter minus next cutter [objectiv R2]. Start {0}.)".format(datetime.datetime.now()))
            else:
                print("(Start mill objectiv mode. MaxBG_down {0} MaxBG_up {1} BG bord. {2} LD {3}. Start {4})".format(self.MaxBackground_down,\
                    self.MaxBackground_up,self.background_border,self.layer_depth,datetime.datetime.now())  )

        if not self.optimize_path and self.roughing_offset > 0 and self.pixelsize <= self.roughing_offset:
            min_len1row = int(self.pixelsize/self.roughing_offset)
        else: 
            min_len1row = 0

        if prn_detail > 0: print("( If rows is only one - min len is {0})".format(min_len1row))

        max_line,max_pix = self.get_RowCol(self.w1,self.h1)

        mas_pieces = []       #the massiv of pieces of all objects on all layers
        mas_glp  = []         #(bonding) gluing (adjacent,nearest) portions of objects in each layer
        tree_prt = {}         #the tree of links between objects of adjacent layers
        mas_rd  = []

        #**********************************************************************************
        #Step 1 - make filter all image
        mas_obj_filter = {}
        jrange = list(range(0, max_line, self.pixelstep))
        if max_line not in jrange: jrange.append(max_line)
        if not self.row_mill: jrange.reverse()  #for cols'''
        for row in jrange:
            mas_obj_filter[row]=(0,max_pix)

        #Step 2 - make preliminary filter without background
        objects_filter = []
        if self.background_border > .0 and self.roughing_minus_finishing:
            
            #This filter is need in first of all - for roughing_minus_finishing:
            #   in RMF we need make G-kode in some places with the background,
            #       but not everywhere where there is background!
            #  Therefore, we first determine where to make g-kod, and where do not make g-kode.
            #  Therefore, in RMF mode - param Max_BG_len is works in a special way.
            #
            #For other types of creating g-kode - "BG filtering" integrated in proc-s: get_entry_pixels_go_XXX, not_background and pixel_use

            if prn_detail > 0: print("( Make filter for background. Max background len: [{0}]. Start at {1})".format(self.max_bg_len,datetime.datetime.now()))
            if self.row_mill: line_cache = 0
            else:       line_cache = self.h1
            object_is_found = True
            object_num = 0
            processed_cur = {}
            tmp_frmf = self.roughing_minus_finishing
            self.roughing_minus_finishing = False
            while object_is_found:

                object_num = object_num + 1
                if prn_detail > 1: print("(  Object_num: {0} line_cache {1})".format(object_num,line_cache))

                make_gk,line_cache,object_is_found,(y,x) = self.get_dict_gk(processed_cur,line_cache,object_num,mas_obj_filter)

                if object_is_found:
                    objects_filter.append((object_num,make_gk))
                    for j in make_gk:
                        Pixel_entry_f,Pixel_entry_l = make_gk[j]
                        # clean covered
                        for i in range(Pixel_entry_f,Pixel_entry_l+1): processed_cur[j,i] = 7

            if prn_detail > 0: 
                print("(End make list of objects filter. Layer: 1 Items: {0}. 'Max bakground len' [{1}])".format(len(objects_filter),self.max_bg_len))

            self.roughing_minus_finishing = tmp_frmf    #restore

            if self.roughing_minus_finishing: 
                self.max_bg_len = max(self.pixelstep, int(self.tool.size/2) ) + 2 # (pixelstep or "radius of cutter") + 2
                if prn_detail > 0: print("(New 'Max bakground len' [{0}])".format(self.max_bg_len))
        else:
            objects_filter.append((1,mas_obj_filter))   #without filtering background = all image
        #**********************************************************************************

        #Step 3 - make list of objects to make gk
        if prn_detail > 0: print("(Make list of objects to make gk. Start at {0})".format(datetime.datetime.now()))
        self.layer = 0
        
        item_npp = 0

        r = .0  #start
        imin = self.image.min()
        if self.roughing_depth == 0: self.roughing_depth = -imin
        self.layer_depth = r

        while r > imin:

            #if self.layer > 3: break
            #if self.layer < 7: continue

            self.layer = self.layer + 1
            
            #step = roughing_depth
            r = max(r - self.roughing_depth,imin)
            if (r - imin) < (self.roughing_depth*roughing_depth_delta): r = imin

            self.layer_depth_prev = self.layer_depth
            self.layer_depth = r
            mas_rd.append(self.layer_depth)

            if prn_detail > 1: print("(  Layer {0} rmd={1} min: {2})".format(self.layer,r,imin))


            processed_cur = {}
            objects_layer = {}    #clear !!!????  not "{}"!!!!!
            object_num = 1
            for object_num_prt,mas_obj_filter in objects_filter:
                
                if self.row_mill:   line_cache = 0          #y
                else:               line_cache = self.h1    #??? x

                object_is_found = True
                while object_is_found:

                    if prn_detail > 1:
                        print(" ")
                        print("(  object_num: {0} paren object {1} line_cache {2})".format(object_num,object_num_prt,line_cache))
                         
                    object_is_found = False
                    make_gk,line_cache,object_is_found,(y,x) = self.get_dict_gk(processed_cur,line_cache,object_num,mas_obj_filter)

                    if object_is_found:

                        if min_len1row > 0 and len(make_gk) == 1 and abs(make_gk[make_gk.keys()[0]][0]-make_gk[make_gk.keys()[0]][1]) <= min_len1row:
                            if prn_detail > 1: print("(  Not added - obj: {0} corse - pixels len: {2} < min len: {3})".format(object_num,object_num_prt,\
                                    abs(make_gk[make_gk.keys()[0]][0]-make_gk[make_gk.keys()[0]][1]),min_len1row))
                            pass
                        else:
                            objects_layer[object_num] = (y,x,make_gk)

                        # clean covered
                        for j in make_gk:
                            Pixel_entry_f,Pixel_entry_l = make_gk[j]
                            for i in range(Pixel_entry_f,Pixel_entry_l+1): processed_cur[j,i] = 7

                        object_num = object_num + 1

            item_npp += len(objects_layer)

            #add all pieces of objects in this layer
            mas_pieces.append(objects_layer)

            #gluing pieces of the object
            #format mas_glp {39: [42], 42: [39,34], 34: [42], 13: [15], 15: [13], 25: [28], 28: [25], 63: [64]}
            mas_glp.append(self.get_mas_glp(objects_layer,self.layer,object_num_prt))



            if prn_detail > 0: print("(End make list of objects. Layers: {0} Items: {1}.)".format(len(mas_pieces),item_npp))

        #Step 4 - group list of objects to tree
        #tree_prt[layer,num child1] = [num parent1,num parent2,num parent3,...]
        if prn_detail > 0: print("(Group list of objects to trees. Start at {0})".format(datetime.datetime.now()))
        tree_prt = self.get_tree_prt(mas_pieces)

        #Step 5 - sorting trees of objects
        if prn_detail > 0: print("(Sorting of all trees. Start at {0})".format(datetime.datetime.now()))
        
        #format processed_obj[layer,num parent] = sequence(traversal) order
        processed_obj = {}        
        while True:

            layer,object_num = self.get_first_piec(mas_pieces,processed_obj)
            if layer < 0 or object_num < 0: break

            processed_obj = self.sort_tree(layer,object_num,mas_pieces,processed_obj,tree_prt,mas_glp,item_npp)

        if prn_detail > 0: print("(End sorting all trees at {0})".format(datetime.datetime.now()))

        #Step 6 - make GK        
        sorted_tree = sorted(processed_obj.items(), key=(lambda k, v: v))
        #print(sorted_tree
        for (layer,object_num),npp in sorted_tree:
            self.layer_depth_prev = self.layer_depth    #'layer_depth_prev' need for 'optimize_path'
            self.layer_depth = mas_rd[layer]
            y,x,make_gk = mas_pieces[layer][object_num]
            self.gk_maker(make_gk, max_pix, {}, convert_scan, primary,object_num,layer)

        if prn_detail > 0: print("(End make g-kod at {0})".format(datetime.datetime.now()))
            
        
    def get_first_piec(self,mas_pieces,processed_obj):

        max_l = ld = len(mas_pieces)
        lrange = range(ld)
        if len(processed_obj) > 0:
            mass = []
            for layer in lrange:
                if layer >= max_l: break
                for object_num in mas_pieces[layer]:
                    try:
                        isprocessed = processed_obj[layer,object_num]
                    except:
                        mass.append((layer,object_num))
                        if self.layer_by_layer: max_l = layer
            layer_last,object_num_last = sorted(processed_obj.items(), key=(lambda k, v: v))[-1][0]        
            layer,object_num = self.get_nearest_obj(-1,-1,layer_last,object_num_last,mas_pieces,mass)

            
            return layer,object_num
        else:
            for layer in lrange:
                for object_num in mas_pieces[layer]:
                    if prn_detail > 1: print("( Start tree from item- L: {0} obj: {1})".format(layer,object_num))
                    return layer,object_num

            return -1,-1

    def is_processed(self,layer,object_num,processed_obj):
        try:
            if processed_obj[layer,object_num] != -1:
                return True
        except:
            pass
        return False

    def get_mas_eb_rekurs(self,layer,object_num,tree_prt,processed_obj,mass):

        #Create an array of 'Ends of the branches'
        #This proc look only Up.
        #get_mas_eb_rekurs - REKURSION

        #*******************************************************************
        #look up for child
        try:
            mas_child = tree_prt[layer][object_num]
        except:
            mas_child = []

        child_is_found = False
        for child_num in mas_child:
            #recurs make child first
            if self.is_processed(layer-1,child_num,processed_obj): continue

            #if prn_detail > 2: print("(  first process his child- L: {0} obj: {1})".format(layer-1,child_num)
            child_is_found = True

            mass = self.get_mas_eb_rekurs(layer-1,child_num,tree_prt,processed_obj,mass)
        #*******************************************************************

        #warning: branches can be fused. Therefore '(layer,object_num) not in mass'
        if not child_is_found and ((layer,object_num) not in mass): 
            mass.append((layer,object_num))
            if prn_detail > 2: print("(   *add end of the branch - L: {0} obj: {1})".format(layer,object_num))


        #*******************************************************************
        #pieces - object (gluing adjacent piece)
        try:
            mas_adj = mas_glp[layer][object_num]
        except:
            mas_adj = []
 
        for cur_p in mas_adj:
            if self.is_processed(layer,cur_p,processed_obj): continue
            if prn_detail > 2: print("(  *add end of the branch in adjacent branch- L: {0} obj: {1})".format(layer,cur_p))

            mass = self.get_mas_eb_rekurs(layer,cur_p,tree_prt,processed_obj,mass)
        #*******************************************************************

        return mass

    def get_nearest_obj(self,layer,object_num,layer_last,object_num_last,mas_pieces,mass):
         
        next_layer,next_object_num = layer,object_num
        if len(mass) < 1:
            pass
        elif len(mass) == 1:
            (next_layer,next_object_num) = mass[0]
        else:
            y,x,make_gk = mas_pieces[layer_last][object_num_last]
            min_len = plus_inf
            for layer2,object_num2 in mass:
                y2,x2,make_gk = mas_pieces[layer2][object_num2]
                #if necessary more speed - use: 'self.find_next_e(mas_p,y,x,max_line,max_pix)' (not 'hypot')
                cur_len = hypot(y2-y,x2-x)
                if min_len > cur_len: 
                    min_len = cur_len
                    next_layer,next_object_num = layer2,object_num2
            if prn_detail > 2: print("(   L: {0} obj: {1} closest to him is L: {2} obj: {3} [{4}])".format(\
                                    layer_last,object_num_last,next_layer,next_object_num,len(mass)))
            
        if next_layer == layer_last and next_object_num == object_num_last:
            if prn_detail > -1: print("(   Error() nearest- not found L: {0} obj: {1} [{2}])".format(next_layer,next_object_num,len(mass)))

        return next_layer,next_object_num


    def sort_tree(self,layer,object_num,mas_pieces,processed_obj,tree_prt,mas_glp,item_npp):

        if prn_detail > 1: print("( look- L: {0} obj: {1})".format(layer,object_num))
        layer_last,object_num_last = layer,object_num
        max_layer = len(mas_pieces)

        #format stek_objs[layer][object_num1] = num_sort
        # stek_objs = {0:{0:0, 0:1, 0:2}, 1:{0:3, 0:4}, 2:{0:5, 1:6}}
        stek_objs = {}

        while True:     #For non-recursive calls

            #object_num - is current piec to process him
            #layer      - is current layer of current piec
            #mas_parents- is parents of pieces
            #stek_objs  - is massive of next pieces(children and adjacent) to process after current piec
            #mas_childs - is massive of children of processed pieces
            #mas_adj    - is massive of adjacent of processed pieces

            if prn_detail > 2: print("( process piec - L: {0} obj: {1})".format(layer,object_num))

            #1. ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            #first of all - look Up for parents (rekursion)
            try:
                mas_parents = tree_prt[layer][object_num]
            except:
                mas_parents = []

            parent_is_found = False
            for parent_num in mas_parents:
                #recurs make parent first
                if self.is_processed(layer-1,parent_num,processed_obj): continue

                if prn_detail > 2: print("(  he has parent/s...)")
                parent_is_found = True


                # format mass = ((layer1,object_num1),(layer2,object_num1),(layer2,object_num2),..)
                mass = self.get_mas_eb_rekurs(layer,object_num,tree_prt,processed_obj,[])

                layer,object_num = self.get_nearest_obj(layer,object_num,layer_last,object_num_last,mas_pieces,mass)

                if prn_detail > 2: print("(  first process his parent- L: {0} obj: {1})".format(layer,object_num))
                break

            if parent_is_found: continue    #process parent
            #^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        


            #2. *******************************************************************
            #make current        
            try:
                isprocessed = processed_obj[layer,object_num]
                if prn_detail > -1: print("(!!!! ERROR(0) ALREDY PROCESSED!!! L: {0} obj: {1})".format(layer,object_num))
                return processed_obj
            except:
                #obj for GK make here

                if prn_detail > 1: print("(  make GK- L: {0} obj: {1})".format(layer,object_num))
                layer_last,object_num_last = layer,object_num
                cur_npp = len(processed_obj)
                processed_obj[layer,object_num] = cur_npp

                try: 
                    stek_objs[layer].remove(object_num)
                    #if prn_detail > 9: print("(& object processed: L: {0} obj: {1})".format(layer,object_num)
                except: 
                    #if prn_detail > 9: print("(& not parent: L: {0} obj: {1})".format(layer,object_num)
                    pass
            #*******************************************************************



            #3.1. --------------------------------------------------------------------------
            #before moving next - remember the children of the current pice
            # If you don't do that before transitions to neighboring - you may be missing some(this) children of current piec...
            # And later will have to do extra returns to missed children.
            if not self.layer_by_layer:
                try:
                    chlds_layer = stek_objs[layer+1]
                except:
                    stek_objs[layer+1] = []
                    chlds_layer = stek_objs[layer+1]


                num_order = len(chlds_layer)

                #add children
                try:
                    mas_childs = tree_prt[layer+1]
                except:
                    mas_childs = []
                #fill array children
                for chl_object_num in mas_childs:
                    if self.is_processed(layer+1,chl_object_num,processed_obj): continue
                    try:
                        mas_parents = mas_childs[chl_object_num]
                    except:
                        mas_parents = []
                    if object_num in mas_parents:
                        #This is children of current piece.
                        if chl_object_num not in chlds_layer:

                            if prn_detail > 2: print("(   remember child- L: {0} obj: {1} order: {2})".format(\
                                                layer+1,chl_object_num,len(chlds_layer)))
                            #order in this array is order to process.
                            chlds_layer.insert(0,chl_object_num)  #append(

                        elif chlds_layer[0] != chl_object_num:
                            #Child alredy added. Typically, a child is closest to the "current"(parent) piec.
                            #If we don't process it in the following cycle, then we will have to come back to it later.
                            #Then we lose time by doing a long way to go back!
                            #Child of current piece mast be next processed. 
                            #Raise the priority of child.
                            if prn_detail > 2: print("(   Raise the priority of children - L: {0} obj: {1} was order: {2})".format(\
                                                layer+1,chl_object_num,chlds_layer.index(chl_object_num)))
                            chlds_layer.remove(chl_object_num)
                            chlds_layer.insert(0,chl_object_num)
                        

                stek_objs[layer+1] = chlds_layer
            #--------------------------------------------------------------------------

            #3.2 ========================================================================
            #before moving further - remember the adjacent pieces of the current layer
            try:
                cur_adj = stek_objs[layer]
            except:
                cur_adj = []
            #add adjacent
            try:
                mas_adj = mas_glp[layer][object_num]
            except:
                mas_adj = []
            for cur_p in mas_adj:
                if self.is_processed(layer,cur_p,processed_obj): continue

                if cur_p not in cur_adj:
                    if prn_detail > 2: print("(   remember adjacent piece- L: {0} obj: {1})".format(layer,cur_p))
                    cur_adj.append(cur_p)

            stek_objs[layer] = cur_adj
            #========================================================================


            if not self.layer_by_layer:
                #4.1 --------------------------------------------------------------------------
                # children of this branch (in added order!!!)
                #mass = []
                child_is_found = False
                for nlayer in range(layer+1,max_layer): #importent: from "layer+1"!
                    try:
                        chlds_layer = stek_objs[nlayer]
                    except:
                        chlds_layer = []
                    for chl_object_num in chlds_layer:
                        if self.is_processed(nlayer,chl_object_num,processed_obj):
                            #if prn_detail > -1: print("(Error() current child alredy processed!? L{0} obj{1})".format(layer,chl_object_num)
                            continue
                        if prn_detail > 2: print("(  process remembered child - L: {0} obj: {1} order: {2})".format(\
                                            nlayer,chl_object_num,chlds_layer.index(chl_object_num)))
                        layer = nlayer
                        object_num = chl_object_num
                        child_is_found = True
                        break
                    if child_is_found: break

                if child_is_found: continue #process child
                #--------------------------------------------------------------------------

                #4.2 --------------------------------------------------------------------------
                # (nearest!?)children of another branches
                child_is_found = False
                for nlayer in range(max_layer,-1,-1):     #instead "max_layer" - correct(better) to use "layer", but reliable to use "max_layer".
                    try:
                        chlds_layer = stek_objs[nlayer]
                    except:
                        chlds_layer = []
                    for chl_object_num in chlds_layer:
                        if self.is_processed(nlayer,chl_object_num,processed_obj):
                            #if prn_detail > -1: print("(Error() current child alredy processed!? L{0} obj{1})".format(layer,chl_object_num)
                            continue
                        if prn_detail > 2: print("(  process remembered child of adj. - L: {0} obj: {1} order: {2} in {3})".format(\
                                            nlayer,chl_object_num,chlds_layer.index(chl_object_num),len(chlds_layer)))
                        layer = nlayer
                        object_num = chl_object_num
                        child_is_found = True
                        break
                    if child_is_found: break

                if child_is_found: continue #process child

                '''        mass.append((nlayer,chl_object_num))
                    if len(mass)>0: break

                if len(mass) > 0:
                    layer,object_num = self.get_nearest_obj(layer_last,object_num_last,layer_last,object_num_last,mas_pieces,mass)
                    if prn_detail > 2: print("(  process remembered - L: {0} obj: {1})".format(layer,chl_object_num)
                    continue'''
            #--------------------------------------------------------------------------


            break   #main exit from 'while'

        if prn_detail > 1: print("( END of tree.)")
        return processed_obj


    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    def Get_first_last_pixels(self, irange, j):
        #This procedure is analogicaly procedures: get_entry_pixels_go_start and get_entry_pixels_go_end, 
        #   but simpler.
        row_y_first = 0;
        row_y_last  = 0;
        for i in irange: #pixels
            y,x = self.get_RowCol(j,i)
            hhh1 = self.get_z_abs(x,y)
            if self.not_background(hhh1,j,i): 
                row_y_last  = i
            else:
                if row_y_last == 0: row_y_first = i
        if row_y_first > 0: row_y_first = row_y_first -1
        if row_y_last < len(irange): row_y_last = row_y_last +1
        return row_y_last, row_y_first

    def mill_rows(self, convert_scan, primary):
        w1 = self.w1; h1 = self.h1
        pixelsize = self.pixelsize; pixelstep = self.pixelstep
        jrange = list(range(0, w1, pixelstep))
        if w1 not in jrange: jrange.append(w1)
        irange = list(range(h1))
        st = 2

        max_j = len(jrange)
        indx_j = 0
        Prev_line = -1
        for j in jrange:    #rows
            # find background--------------------------------------
            st = st + 1
            row_y_first = 0
            if self.background_border == 0:
               row_y_last  = irange[-1];
            else:
                row_y_last  = 0;

                #********************************
                row_y_last, row_y_first = self.Get_first_last_pixels(irange, j)
                #********************************
  
                #dont_cut_angles:
                # find new row MaxBackgrounds
                row_y_first_next = irange[-1];
                row_y_last_next  = 0;
                if (indx_j+1) < max_j:
                    j_next = jrange[indx_j+1];
                    #********************************
                    row_y_last_next, row_y_first_next = self.Get_first_last_pixels(irange, j_next)
                    #********************************

                # find previous row MaxBackgrounds
                row_y_first_prev = irange[-1];
                row_y_last_prev  = 0;
                if Prev_line > 0:

                    #********************************
                    row_y_last_prev, row_y_first_prev = self.Get_first_last_pixels(irange, Prev_line)
                    #********************************
            
                if st % 2:
                    if row_y_last_prev > row_y_last: row_y_last = row_y_last_prev
                    if row_y_first_next < row_y_first: row_y_first = row_y_first_next
                else:
                    if row_y_last_next > row_y_last: row_y_last = row_y_last_next
                    if row_y_first_prev < row_y_first: row_y_first = row_y_first_prev
                #-dont_cut_angles
            #--------------------------------------
            indx_j = indx_j + 1
            
            y = (w1-j+self.ts2) * pixelsize
            scan = []
            for i in irange:
                if i >= row_y_first and i<= row_y_last:
                    x = (i+self.ts2) * pixelsize
                    milldata = (i, (x, y, self.get_z(i, j)),
                        self.get_dz_dx(i, j), self.get_dz_dy(i, j))
                    scan.append(milldata)

            # make g-code
            if len(scan) != 0:
              for flag, points in convert_scan(primary, scan):
                if flag:
                    self.entry_cut(self, points[0][0], j, points)
                for p in points:
                    self.g.cut(*p[1])
            else:
                st = st - 1
            self.g.flush()
            Prev_line = j

    def mill_cols(self, convert_scan, primary):
        w1 = self.w1; h1 = self.h1;
        pixelsize = self.pixelsize; pixelstep = self.pixelstep
        jrange = range(0, h1, pixelstep)
        irange = range(w1)
        if h1 not in jrange: jrange.append(h1)
        jrange.reverse()
        st = 0

        max_j = len(jrange)
        indx_j = 0
        Prev_line = -1
        for j in jrange:    #cols
            # find background--------------------------------------
            st = st + 1
            row_y_first = 0;
            if self.background_border == 0:
               row_y_last  = irange[-1];
            else:
                row_y_last  = 0;

                #********************************
                row_y_last, row_y_first = self.Get_first_last_pixels(irange, j)
                #********************************
  
                #dont_cut_angles:
                # find new col MaxBackgrounds
                row_y_first_next = irange[-1];
                row_y_last_next  = 0;
                if (indx_j+1) < max_j:
                    j_next = jrange[indx_j+1];
                    #********************************
                    row_y_last_next, row_y_first_next = self.Get_first_last_pixels(irange, j_next)
                    #********************************

                # find previous col MaxBackgrounds
                row_y_first_prev = irange[-1];
                row_y_last_prev  = 0;
                if Prev_line > 0:

                   #********************************
                   row_y_last_prev, row_y_first_prev = self.Get_first_last_pixels(irange, Prev_line)
                   #********************************
            
                if st % 2:
                    if row_y_last_prev > row_y_last: row_y_last = row_y_last_prev
                    if row_y_first_next < row_y_first: row_y_first = row_y_first_next
                else:
                    if row_y_last_next > row_y_last: row_y_last = row_y_last_next
                    if row_y_first_prev < row_y_first: row_y_first = row_y_first_prev
                #-dont_cut_angles
            #--------------------------------------
            indx_j = indx_j + 1

            x = (j+self.ts2) * pixelsize
            scan = []
            for i in irange:
                if i >= row_y_first and i<= row_y_last:
                    y = (w1-i+self.ts2) * pixelsize
                    milldata = (i, (x, y, self.get_z(j, i)),
                      self.get_dz_dy(j, i), self.get_dz_dx(j, i))
                    scan.append(milldata)

            # make g-code
            if len(scan) != 0:
              for flag, points in convert_scan(primary, scan):
                if flag:
                    self.entry_cut(self, j, points[0][0], points)
                for p in points:
                    self.g.cut(*p[1])
            else:
                st = st - 1
            self.g.flush()
            Prev_line = j

def convert(*args, **kw):
    return Converter(*args, **kw).convert()

options = dict(
    invert = False,
    normalize = False,
    expand = 0,
    pixel_size = .006,
    depth = 0.25,
    background_border = .0,
    max_bg_len = 1,
    pixelstep = 8,
    safety_height = .012,
    tool_diameter = 1/16.,
    tool_type = 0,
    tool_diameter2 = 1/16.,
    angle2 = .0,
    tolerance = .001,
    feed_rate = 12,
    plunge_feed_rate = 12,
    units = 0,
    pattern = 0,
    converter = 0,
    bounded = 0,
    contact_angle = 45,
    spindle_speed = 1000,
    layer_by_layer = False,
    optimize_path = False,
    roughing_minus_finishing = False,
    min_delta_rmf = .0,
    previous_offset = .1,
    roughing_offset = .1,
    roughing_depth = .25,
    pixelstep_roughing = 8,
    tool_diameter_roughing = 1/16.,
    tool_type_roughing = 0,
    tool_diameter_roughing2 =1/16.,
    angle2_roughing=.0,
    cut_top_jumper = False,
    detail_of_comments = 0
)

def gray2gcode(im_name):
    from PIL import Image

    im = Image.open(im_name)
    size = im.size
    im = im.convert("L") #grayscale
    w, h = im.size
    try:
      nim = np.frombuffer(im.tobytes(), dtype=np.uint8)
    except AttributeError:
      nim = np.frombuffer(im.tobytes(), dtype=np.uint8)
    nim = nim.reshape(w,h)
    
    newDir = "nc_output"
    fileExtention = ".nc"
    fileName = "gcodeout"
    if not os.path.isdir(newDir):
        os.makedirs(newDir)      
    finalFileName = fileName+str(datetime.datetime.now().strftime("_%d%m%Y_%H.%M.%S"))+fileExtention
    if platformWin:
        sys.stdout = open(newDir+"\\"+finalFileName, 'w')
    else:
        sys.stdout = open(newDir+"/"+finalFileName, 'w')
    
    step = options['pixelstep']
    depth = options['depth']

    tool_info(options['tool_type'], options['tool_diameter'], options['tool_diameter2'], options['angle2'], options['units'])

    if options['normalize']:
        a = nim.min()
        b = nim.max()
        if a != b:
            nim = (nim - a) / (b-a)
    else:
        nim = nim / 255.0

    maker = tool_makers[options['tool_type']]
    tool_diameter = options['tool_diameter']
    pixel_size = options['pixel_size']
    tool = make_tool_shape(maker, tool_diameter, pixel_size,False,options['tool_type'],options['tool_diameter2'],options['angle2'])
    tool = optim_tool_shape(tool,depth,options['roughing_offset'],max(options['pixel_size'],options['tolerance']))

    if options['expand']:
        if options['expand'] == 1: pixel = 1
        else: pixel = 0
        w, h = nim.shape
        tw, th = tool.shape
        w1 = w + 2*tw
        h1 = h + 2*th
        nim1 = np.zeros((w1, h1), dtype=np.float32) + pixel
        nim1[tw:tw+w, th:th+h] = nim
        nim = nim1
        w, h = w1, h1
    nim = nim * depth

    if options['invert']:
        nim = -nim
    else:
        nim = nim - depth

    print("(Image max= {0} min={1})".format(nim.max(),nim.min()))

    rows = options['pattern'] != 1 and options['pattern'] != 5
    columns = options['pattern'] != 0 and options['pattern'] != 4
    columns_first = options['pattern'] == 3
    pattern_objectiv = options['pattern'] >= 4 and options['pattern'] <= 5
    spindle_speed = options['spindle_speed']
    if rows: convert_rows = convert_makers[options['converter']]()
    else: convert_rows = None
    if columns: convert_cols = convert_makers[options['converter']]()
    else: convert_cols = None

    if options['bounded'] and rows and columns:
        slope = tan(((180-options['contact_angle'])/2) * pi / 180)
        if columns_first:
            convert_rows = Reduce_Scan_Lace(convert_rows, slope, step+1)
        else:
            convert_cols = Reduce_Scan_Lace(convert_cols, slope, step+1)
        if options['bounded'] > 1:
            if columns_first:
                convert_cols = Reduce_Scan_Lace(convert_cols, slope, step+1)
            else:
                convert_rows = Reduce_Scan_Lace(convert_rows, slope, step+1)

    maker_roughing = tool_makers[options['tool_type_roughing']]
    tool_diameter_roughing = options['tool_diameter_roughing']
    tool_roughing = make_tool_shape(maker_roughing, tool_diameter_roughing, pixel_size,False,options['tool_type_roughing'],options['tool_diameter_roughing2'],options['angle2_roughing'])
    tool_roughing = optim_tool_shape(tool_roughing,depth,options['roughing_offset'],max(options['pixel_size'],options['tolerance']))

    units = unitcodes[options['units']]
    convert(nim, units, tool, pixel_size, step,
        options['safety_height'], options['tolerance'], options['feed_rate'],
        convert_rows, convert_cols, columns_first, ArcEntryCut(options['plunge_feed_rate'], .125),
        spindle_speed, options['roughing_offset'], options['roughing_depth'], options['feed_rate'],
        options['background_border'],options['cut_top_jumper'],
        options['optimize_path'],options['layer_by_layer'],options['max_bg_len'],
        pattern_objectiv,
        options['roughing_minus_finishing'],options['pixelstep_roughing'],tool_roughing,options['min_delta_rmf'],options['previous_offset'])
    
    return finalFileName
