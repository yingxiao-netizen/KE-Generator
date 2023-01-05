import ezdxf
from ezdxf import units
import pandas as pd
import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry.polygon import LinearRing
from shapely.ops import linemerge

########################## ASSET STANDARDS AUTHORITY SYDNEY TRAINS ##########################
def ASAKEenve(Ea, stdgauge, rad, track, RSoutline, location):
    # Setup dxf model space and process rolling stock outline

    def EaMax(Ea, stdgauge):
        a = -np.arctan((Ea / (stdgauge * 2)))
        x = stdgauge
        y = 0
        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx


    def RSTolLatPl(Ea, stdgauge):
        tol = 60
        x = tol
        y = 0

        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSTolLatNg(Ea, stdgauge):
        tol = -60
        x = tol
        y = 0

        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSTolVert():
        tol = 50
        x = 0
        y = tol
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSRotCCW(RollCen):
        a = np.deg2rad(2)
        x = RollCen[0]
        y = RollCen[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])

        return mx


    def RSRotCW(RollCen):
        a = np.deg2rad(-2)
        x = RollCen[0]
        y = RollCen[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])

        return mx


    def trklatPl(track, rad):
        if track == 'TSBT' and rad > 2000000:
            x = 25 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
#rail wear+latral horizontal curves >2000m radius and tangent track
        if track == 'TSBT' and rad <= 2000000:
            x = 35 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
#TSBT(TIMBER SLEEPERED BALLASTED TRACK)
        if track == 'CST' and rad > 2000000:
            x = 15 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'CST' and rad <= 2000000:
            x = 25 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
#CST(CONCRETE SLAB TRACK)
        if track == 'STTTB' and rad > 2000000:
            x = 10 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'STTTB' and rad <= 2000000:
            x = 20 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
#STTTB(SLAB TRACK AND TRANSOM TOP BRIDGES)

    def trklatNg(track, rad):
        if track == 'TSBT' and rad > 2000000:
            x = -(25 + 15)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'TSBT' and rad <= 2000000:
            x = -(35 + 15)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

        if track == 'CST' and rad > 2000000:
            x = -(15 + 15)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'CST' and rad <= 2000000:
            x = -(25 + 15)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

        if track == 'STTTB' and rad > 2000000:
            x = -(10 + 15)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'STTTB' and rad <= 2000000:
            x = -(20 + 15)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

#Track vertical alignment tolerances (positive)
    def trkvertPl(track):
        if track == 'TSBT' or 'CST':
            x = 0
            y = 100
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'STTTB':
            x = 0
            y = 50
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

#Track vertical alignment tolerances (negative)
    def trkvertNg(track):
        if track == 'TSBT' or 'CST':
            x = 0
            y = -150
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'STTTB':
            x = 0
            y = -50
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

#Track rotational superelevation (positive)
    def trkEaPl(stdgauge):
        Ea = 10
        a = -np.arcsin((Ea / (stdgauge * 2)))
        x = stdgauge
        y = 0
        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx

#Track rotational superelevation (negative)
    def trkEaNg(stdgauge):
        Ea = -10
        a = -np.arcsin((Ea / (stdgauge * 2)))
        x = stdgauge
        y = 0
        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx

#super elevation tolerance
    def ctrthrow(ctrthr):
        x = ctrthr + 5
        y = 0
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        return mx


    def endthrow(endthr):
        x = -endthr - 5
        y = 0
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        return mx


    def mirror():
        mx = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
        return mx


    def strgauge(rad, Ea, RSoutline, ctrthr, endthr):
        from re import search
        Z = 3800
        #Z=the height above design rail height
        B = 5400
        #B=vertical clearance required (PUBLIC LEVEL CROSSINGS)
        strgage = [[0, 0, 1],
                   [2060 + ctrthr + Ea * Z / 1435, 0, 1],
                   [2060 + ctrthr + Ea * Z / 1435, Z, 1],
                   [1525 + ctrthr + Ea * B / 1435, B, 1],
                   [0, B, 1],
                   [-1525 - endthr + Ea * B / 1435, B, 1],
                   [-2060 - endthr + Ea * Z / 1435, Z, 1],
                   [-2060 - endthr + Ea * Z / 1435, 0, 1],
                   [0, 0, 1]]
        return strgage

    RollCen = np.array([0, 610, 1])
    doc = ezdxf.new()
    doc.units = units.MM
    RS_CS = pd.read_csv(RSoutline)
    RS_CS['X'] = RS_CS['X'].astype('float')
    RS_CS['Y'] = RS_CS['Y'].astype('float')
    bogctr = RS_CS['X'][0]
    bodovr = RS_CS['X'][1]
    vecwid = RS_CS['X'][2]

    doc = ezdxf.new(setup=True)
    msp = doc.modelspace()  # add new entities to the modelspace
    # dxf layers list

    doc.layers.new(name='Rolling Stock OL', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='Rolling Stock on Ea', dxfattribs={'linetype': 'CONTINUOUS', 'color': 1})

    doc.layers.new(name='RS&Ea&Eatol', dxfattribs={'linetype': 'CONTINUOUS', 'color': 2})

    doc.layers.new(name='RS&Ea&Eatol&roll', dxfattribs={'linetype': 'CONTINUOUS', 'color': 3})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol', dxfattribs={'linetype': 'CONTINUOUS', 'color': 4})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol&trkalign', dxfattribs={'linetype': 'CONTINUOUS', 'color': 5})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr', dxfattribs={'linetype': 'CONTINUOUS', 'color': 6})

    doc.layers.new(name='KE200', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='Structural Gauge', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='KE', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    # Data setup

    RS_R = np.transpose([np.array(RS_CS.iloc[4:RS_CS.shape[0], 1]), np.array(RS_CS.iloc[4:RS_CS.shape[0], 2]),
                         np.ones(RS_CS.shape[0] - 4)])
    RS_L = np.transpose([np.array(-RS_CS.iloc[4:RS_CS.shape[0], 1]), np.array(RS_CS.iloc[4:RS_CS.shape[0], 2]),
                         np.ones(RS_CS.shape[0] - 4)])
    RS = np.append(RS_R, RS_L[::-1], axis=0)

    if rad == 0:
        ctrthr = 0
    if rad > 0 or rad < 0:
        ctrthr = (bogctr ** 2) / (8 * np.abs(rad))
    if rad == 0:
        endthr = 0
    if rad > 0 or rad < 0:
        endthr = ((((bogctr + 2 * bodovr) ** 2) / (8 * np.abs(rad) + 4 * vecwid))) - ctrthr
    # EA applied to RS points
    EaRS = [np.array(EaMax(Ea, stdgauge).dot(RS[n])) for n in range(len(RS))]

    EaRC = np.array(EaMax(Ea, stdgauge).dot(RollCen))

    EaRCzero = np.array(EaMax(Ea, stdgauge).dot([0, 0, 1]))

    # Draw Extreme Right Horizontal Inside Curve

    # Draw RS Outline
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkEaPl(stdgauge).dot(EaRC)
        msp.add_lwpolyline([trkEaPl(stdgauge).dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline([RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkEaPl(stdgauge).dot(EaRC))

        msp.add_lwpolyline([mirror().dot(trkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [mirror().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([mirror().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left Horizontal Outside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkEaNg(stdgauge).dot(EaRC)
        msp.add_lwpolyline([trkEaNg(stdgauge).dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline([RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline([mirror().dot((EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkEaNg(stdgauge).dot(EaRC))

        msp.add_lwpolyline([mirror().dot(trkEaNg(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [mirror().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([mirror().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC))))
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(
            trklatPl(track, rad).dot(
                RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left UP VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot((EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC))))

        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(
            trklatNg(track, rad).dot(
                RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC))

        msp.add_lwpolyline([trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([mirror().dot(
            trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))
                            for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left DOWN VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC))

        msp.add_lwpolyline([trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC)))

        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(
            trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))
                            for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ## Setup Convex Hull
    # Draw Extreme Right Horizontal Inside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkEaPl(stdgauge).dot(EaRC)
        xi1 = [[RS[n]] for n in range(len(RS) - 1)]
        xi2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xi3 = [[trkEaPl(stdgauge).dot(EaRS[n])] for n in range(len(RS) - 1)]
        xi4 = [[RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xi5 = [[RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xi6 = [[trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
            range(len(RS) - 1)]
        xi7 = [[ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkEaPl(stdgauge).dot(EaRC))
        i1 = [[RS[n]] for n in range(len(RS) - 1)]
        i2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        i3 = [[mirror().dot(trkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        i4 = [[mirror().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        i5 = [[mirror().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n
              in range(len(RS) - 1)]
        i6 = [[mirror().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        i7 = [[mirror().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        i = i1 + i2 + i3 + i4 + i5 + i6 + i7 + xi1 + xi2 + xi3 + xi4 + xi5 + xi6 + xi7
    else:
        try:
            i = i1 + i2 + i3 + i4 + i5 + i6 + i7
        except:
            pass
        try:
            i = xi1 + xi2 + xi3 + xi4 + xi5 + xi6 + xi7
        except:
            pass
    ai = np.array([i[n][0][:].tolist() for n in range(len(i))])

    # Draw Extreme Left Horizontal Outside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkEaNg(stdgauge).dot(EaRC)
        xa1 = [[RS[n]] for n in range(len(RS) - 1)]
        xa2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xa3 = [[trkEaNg(stdgauge).dot(EaRS[n])] for n in range(len(RS) - 1)]
        xa4 = [[RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xa5 = [[RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xa6 = [[trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
            range(len(RS) - 1)]
        xa7 = [[endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkEaNg(stdgauge).dot(EaRC))
        a1 = [[RS[n]] for n in range(len(RS) - 1)]
        a2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        a3 = [[mirror().dot(trkEaNg(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        a4 = [[mirror().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        a5 = [[mirror().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n
              in range(len(RS) - 1)]
        a6 = [[mirror().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        a7 = [[mirror().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        a = a1 + a2 + a3 + a4 + a5 + a6 + a7 + xa1 + xa2 + xa3 + xa4 + xa5 + xa6 + xa7
    else:
        try:
            a = a1 + a2 + a3 + a4 + a5 + a6 + a7
        except:
            pass
        try:
            a = xa1 + xa2 + xa3 + xa4 + xa5 + xa6 + xa7
        except:
            pass
    aa = np.array([a[n][0][:].tolist() for n in range(len(a))])

    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC)))
        xb1 = [[RS[n]] for n in range(len(RS) - 1)]
        xb2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xb3 = [[trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xb4 = [[trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
               range(len(RS) - 1)]
        xb5 = [[trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n
            in range(len(RS) - 1)]
        xb6 = [[trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        xb7 = [[trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC))))
        b1 = [[RS[n]] for n in range(len(RS) - 1)]
        b2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        b3 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        b4 = [
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        b5 = [[mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n
            in range(len(RS) - 1)]
        b6 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
        b7 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        b = b1 + b2 + b3 + b4 + b5 + b6 + b7 + xb1 + xb2 + xb3 + xb4 + xb5 + xb6 + xb7
    else:
        try:
            b = b1 + b2 + b3 + b4 + b5 + b6 + b7
        except:
            pass
        try:
            b = xb1 + xb2 + xb3 + xb4 + xb5 + xb6 + xb7
        except:
            pass
    bb = np.array([b[n][0][:].tolist() for n in range(len(b))])

    # Draw Extreme Left UP VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC)))
        xc1 = [[RS[n]] for n in range(len(RS) - 1)]
        xc2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xc3 = [[trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xc4 = [[trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
               range(len(RS) - 1)]
        xc5 = [[trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n
            in range(len(RS) - 1)]
        xc6 = [[trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        xc7 = [[trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC))))
        c1 = [[RS[n]] for n in range(len(RS) - 1)]
        c2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        c3 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        c4 = [
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        c5 = [[mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for
            n in range(len(RS) - 1)]
        c6 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
        c7 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))))] for n in
            range(len(RS) - 1)]

    if Ea == 0:
        c = c1 + c2 + c3 + c4 + c5 + c6 + c7 + xc1 + xc2 + xc3 + xc4 + xc5 + xc6 + xc7
    else:
        try:
            c = c1 + c2 + c3 + c4 + c5 + c6 + c7
        except:
            pass
        try:
            c = xc1 + xc2 + xc3 + xc4 + xc5 + xc6 + xc7
        except:
            pass
    cc = np.array([c[n][0][:].tolist() for n in range(len(c))])

    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC))
        xd1 = [[RS[n]] for n in range(len(RS) - 1)]
        xd2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xd3 = [[trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xd4 = [[trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xd5 = [
            [trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))]
            for n in range(len(RS) - 1)]
        xd6 = [[trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        xd7 = [[trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC)))
        d1 = [[RS[n]] for n in range(len(RS) - 1)]
        d2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        d3 = [[mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        d4 = [[mirror().dot(trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        d5 = [[mirror().dot(
            trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        d6 = [[mirror().dot(trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        d7 = [[mirror().dot(trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        d = d1 + d2 + d3 + d4 + d5 + d6 + d7 + xd1 + xd2 + xd3 + xd4 + xd5 + xd6 + xd7
    else:
        try:
            d = d1 + d2 + d3 + d4 + d5 + d6 + d7
        except:
            pass
        try:
            d = xd1 + xd2 + xd3 + xd4 + xd5 + xd6 + xd7
        except:
            pass
    dd = np.array([d[n][0][:].tolist() for n in range(len(d))])

    # Draw Extreme Left DOWN VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC))
        xe1 = [[RS[n]] for n in range(len(RS) - 1)]
        xe2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xe3 = [[trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xe4 = [[trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xe5 = [
            [trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))]
            for n in range(len(RS) - 1)]
        xe6 = [[trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        xe7 = [[trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC)))
        e1 = [[RS[n]] for n in range(len(RS) - 1)]
        e2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        e3 = [[mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        e4 = [[mirror().dot(trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        e5 = [[mirror().dot(
            trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        e6 = [[mirror().dot(trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        e7 = [[mirror().dot(trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        e = e1 + e2 + e3 + e4 + e5 + e6 + e7 + xe1 + xe2 + xe3 + xe4 + xe5 + xe6 + xe7
    else:
        try:
            e = e1 + e2 + e3 + e4 + e5 + e6 + e7
        except:
            pass
        try:
            e = xe1 + xe2 + xe3 + xe4 + xe5 + xe6 + xe7
        except:
            pass
    ee = np.array([e[n][0][:].tolist() for n in range(len(e))])

    ii = np.concatenate((ai, aa, bb, cc, dd, ee))

    conai = np.array([ii[n].tolist() for n in range(len(ii))])
    ConHu = ConvexHull(np.delete(conai, 2, 1)).vertices
    OutLConHu = ii[ConHu]
    msp.add_lwpolyline(np.append(np.array(OutLConHu), [np.array(OutLConHu)[0]], axis=0), dxfattribs={'layer': 'KE'})

    h = []
    for n in range(len(ConHu)):
        h.append([ii[ConHu, 0][n], ii[ConHu, 1][n]])
    poly_line = LinearRing(h)
    poly_line_offset = poly_line.parallel_offset(200, 'right', join_style=2, mitre_limit=5)
    ke200 = list(poly_line_offset.convex_hull.boundary.coords)
    msp.add_lwpolyline(np.append(np.array(ke200), [np.array(ke200)[0]], axis=0), dxfattribs={'layer': 'KE200'})

    strgage1 = strgauge(rad, Ea, RSoutline, ctrthr, endthr)
    strgage = []

    if rad > 0 or rad == 0:
        strgage = strgauge(rad, Ea, RSoutline, ctrthr, endthr)
    if rad < 0:
        for n in range(len(strgage1) - 1):
            strgage.append(mirror().dot(strgauge(rad, Ea, RSoutline, ctrthr, endthr)[n]))

    msp.add_lwpolyline(np.append(np.array(strgage), [np.array(strgage)[0]], axis=0),
                       dxfattribs={'layer': 'Structural Gauge'})
    msp.add_text(str(location),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1000), align='MIDDLE_LEFT')

    msp.add_text(str(RSoutline),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1250), align='MIDDLE_LEFT')

    msp.add_text("Cant: " + str(Ea) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1500), align='MIDDLE_LEFT')

    msp.add_text("Curve Radius: " + str(rad / 1000) + "m",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1750), align='MIDDLE_LEFT')

    msp.add_text("Centre Throw: " + str(round(ctrthr, 2)) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2000), align='MIDDLE_LEFT')

    msp.add_text("End Throw: " + str(round(endthr, 2)) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2250), align='MIDDLE_LEFT')

    msp.add_text("Track Tolerances",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2750), align='MIDDLE_LEFT')

    msp.add_text("Track Type: " + str(track),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3000), align='MIDDLE_LEFT')

    msp.add_text("Rail Wear: " + str(15) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3250), align='MIDDLE_LEFT')

    msp.add_text("Lateral Alignment: " + str(trklatPl(track, rad)[0][2] - 15) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3500), align='MIDDLE_LEFT')

    msp.add_text("Superelevation: " + str(10) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3750), align='MIDDLE_LEFT')

    msp.add_text("Vertical Positive: " + str(trkvertPl(track)[1][2]) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4000), align='MIDDLE_LEFT')

    msp.add_text("Vertical Negative: " + str(-trkvertNg(track)[1][2]) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4250), align='MIDDLE_LEFT')

    msp.add_text("Rolling Stock Tolerances",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4750), align='MIDDLE_LEFT')

    msp.add_text("Lateral: " + str(60) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5000), align='MIDDLE_LEFT')

    msp.add_text("Vertical: " + str(50) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5250), align='MIDDLE_LEFT')

    msp.add_text("Roll: " + str(2) + " degrees",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5500), align='MIDDLE_LEFT')

    msp.add_line((0, 0), (0, 6000))

    doc.saveas(location + '.dxf')

########################## ASSET STANDARDS AUTHORITY SYDNEY TRAINS ##########################

########################## SYDNEY METRO #####################################################
def SMKEenve(Ea, stdgauge, rad, track, RSoutline, location):
    # Setup dxf model space and process rolling stock outline

    def EaMax(Ea, stdgauge):
        a = -np.arctan((Ea / (stdgauge * 2)))
        x = stdgauge
        y = 0
        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx


    def RSTolLatPl(Ea, stdgauge):
        tol = 44
        x = tol
        y = 0

        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSTolLatNg(Ea, stdgauge):
        tol = -44
        x = tol
        y = 0

        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSTolVert():
        tol = 50
        x = 0
        y = tol
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSRotCCW(RollCen):
        a = np.deg2rad(2)
        x = RollCen[0]
        y = RollCen[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])

        return mx


    def RSRotCW(RollCen):
        a = np.deg2rad(-2)
        x = RollCen[0]
        y = RollCen[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])

        return mx

#Track lateral alignment tolerances (positive)
    def trklatPl(track, rad):
#BT(BALLASTED TRACK)
        if track == 'SM_BT' and rad > 2000000:
            x = 15 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
#rail wear+latral horizontal curves >2000m radius and tangent track
        if track == 'SM_BT' and rad <= 2000000:
            x = 25 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
#rail wear+ horizontal curves <2000m radius
#CST(CONCRETE SLAB TRACK)
        if track == 'SM_CST' and rad > 2000000:
            x = 10 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'SM_CST' and rad <= 2000000:
            x = 20 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

#Track lateral alignment tolerances (negative)
    def trklatNg(track, rad):
        if track == 'SM_BT' and rad > 2000000:
            x = -15 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'SM_BT' and rad <= 2000000:
            x = -25 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

        if track == 'SM_CST' and rad > 2000000:
            x = -10 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'SM_CST' and rad <= 2000000:
            x = -20 + 15
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

#Track vertical alignment tolerances (positive)
    def trkvertPl(track):
        if track == 'SM_BT':
            x = 0
            y = 50
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'SM_CST':
            x = 0
            y = 20
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

#Track vertical alignment tolerances (negative)
    def trkvertNg(track):
        if track == 'SM_BT':
            x = 0
            y = -50
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx
        if track == 'SM_CST':
            x = 0
            y = -20
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

#Track rotational superelevation (positive)
    def trkEaPl(stdgauge):
        if track == 'SM_BT':
            Ea = 10
            a = -np.arcsin((Ea / (stdgauge * 2)))
            x = stdgauge
            y = 0
            mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
            [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
            return mx
        if track == 'SM_CST':
            Ea = 8
            a = -np.arcsin((Ea / (stdgauge * 2)))
            x = stdgauge
            y = 0
            mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
            [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
            return mx

#Track rotational superelevation (negative)
    def trkEaNg(stdgauge):
        if track == 'SM_BT':
            Ea = -10
            a = -np.arcsin((Ea / (stdgauge * 2)))
            x = stdgauge
            y = 0
            mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
            [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
            return mx
        if track == 'SM_CST':
            Ea = -8
            a = -np.arcsin((Ea / (stdgauge * 2)))
            x = stdgauge
            y = 0
            mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
            [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
            return mx

#M value = a centre-throw and end-throw component as detailed in ASA ESC215 section 7.1.2
# add 5mm each way to allow for extra tolerance 
    def ctrthrow(ctrthr):
        x = ctrthr + 5
        y = 0
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        return mx

    def endthrow(endthr):
        x = -endthr - 5
        y = 0
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        return mx

#mirroring the other side
    def mirror():
        mx = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
        return mx


    def strgauge(rad, Ea, RSoutline, ctrthr, endthr):
        from re import search
        Z = 3400 
        #Z=the height above design rail height
        H = 5950
        #H=vertical clearance required (sepcified on the diagram)
        strgage = [[0, 0, 1],
                   [2060 + ctrthr, 0, 1],
                   [2060 + ctrthr, Z, 1],
                   [1525 + ctrthr, H, 1],
                   [0, H, 1],
                   [-1525 - endthr, H, 1],
                   [-2060 - endthr, Z, 1],
                   [-2060 - endthr, 0, 1],
                   [0, 0, 1]]
        return strgage

#RollCen=rolling centre
    RollCen = np.array([0, 610, 1])
    doc = ezdxf.new()
    doc.header['$INSUNITS'] = 4
    RS_CS = pd.read_csv(RSoutline)
    RS_CS['X'] = RS_CS['X'].astype('float')
    RS_CS['Y'] = RS_CS['Y'].astype('float')
    bogctr = RS_CS['X'][0]
    bodovr = RS_CS['X'][1]
    vecwid = RS_CS['X'][2]
# create model space and import list of layers we want to create
    doc = ezdxf.new(setup=True)
    msp = doc.modelspace()  # add new entities to the modelspace
    # dxf layers list

    doc.layers.new(name='Rolling Stock OL', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='Rolling Stock on Ea', dxfattribs={'linetype': 'CONTINUOUS', 'color': 1})

    doc.layers.new(name='RS&Ea&Eatol', dxfattribs={'linetype': 'CONTINUOUS', 'color': 2})

    doc.layers.new(name='RS&Ea&Eatol&roll', dxfattribs={'linetype': 'CONTINUOUS', 'color': 3})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol', dxfattribs={'linetype': 'CONTINUOUS', 'color': 4})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol&trkalign', dxfattribs={'linetype': 'CONTINUOUS', 'color': 5})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr', dxfattribs={'linetype': 'CONTINUOUS', 'color': 6})

    doc.layers.new(name='KE200', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='Structural Gauge', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='KE', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    # Data setup for the original rolling stock outline 

    RS_R = np.transpose([np.array(RS_CS.iloc[4:RS_CS.shape[0], 1]), np.array(RS_CS.iloc[4:RS_CS.shape[0], 2]),
                         np.ones(RS_CS.shape[0] - 4)])
    RS_L = np.transpose([np.array(-RS_CS.iloc[4:RS_CS.shape[0], 1]), np.array(RS_CS.iloc[4:RS_CS.shape[0], 2]),
                         np.ones(RS_CS.shape[0] - 4)])
    RS = np.append(RS_R, RS_L[::-1], axis=0)

#this part is calculating the M value,centre-throw and end-throw component values, which is the same in this case (M=27565/R) 
    if rad == 0:
        ctrthr = 0
    if rad > 0 or rad < 0:
        ctrthr = 27565 / (np.abs(rad)/1000)
    if rad == 0:
        endthr = 0
    if rad > 0 or rad < 0:
        endthr = 27565 / (np.abs(rad)/1000)
    
    # EA applied to RS points
    #RS=Rolloing Stock, EA=super elevation
    EaRS = [np.array(EaMax(Ea, stdgauge).dot(RS[n])) for n in range(len(RS))]
# EA applied to Rolling centre point
    EaRC = np.array(EaMax(Ea, stdgauge).dot(RollCen))
# EA applied to Rolling centre point, zero, the bottom of train
    EaRCzero = np.array(EaMax(Ea, stdgauge).dot([0, 0, 1]))
######################################################################################################################################################################################################################
#here we create a different set of code just for the staright track, it is plotted according to the coordinates provided by SM, so there is no calculation
#define KE_OUTLINE

    KE_OUTLINE = []

    # Draw RS Outline if (rad=0), Track is straight
    if rad == 0 and Ea == 0:
        #plot rolling stock outline
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        #plot rolling rollingstock on elevation 
        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                        dxfattribs={'layer': 'Rolling Stock on Ea'})
        # KE envelope
        if track == 'SM_CST':
            KE_OUT = pd.read_csv('STRAIGHT_KE_CST.csv')
            KE_OUT['X'] = KE_OUT['X'].astype('float')
            KE_OUT['Y'] = KE_OUT['Y'].astype('float')
        
        if track == 'SM_BT':
            KE_OUT = pd.read_csv('STRAIGHT_KE_BT.csv')
            KE_OUT['X'] = KE_OUT['X'].astype('float')
            KE_OUT['Y'] = KE_OUT['Y'].astype('float')

    # Data setup
# KE outline
        KE_R = np.transpose([np.array(KE_OUT.iloc[4:KE_OUT.shape[0], 1]), np.array(KE_OUT.iloc[4:KE_OUT.shape[0], 2]),
                         np.ones(KE_OUT.shape[0] - 4)])
        KE_L = np.transpose([np.array(-KE_OUT.iloc[4:KE_OUT.shape[0], 1]), np.array(KE_OUT.iloc[4:KE_OUT.shape[0], 2]),
                         np.ones(KE_OUT.shape[0] - 4)])
        KE_OUTLINE = np.append(KE_R, KE_L[::-1], axis=0)

        msp.add_lwpolyline(KE_OUTLINE, dxfattribs={'layer': 'KE'})
        
        h = []

        for n in range(len(KE_OUTLINE)):
           h.append([KE_OUTLINE[n][0], KE_OUTLINE[n][1]])
        poly_line = LinearRing(h)
        poly_line_offset = poly_line.parallel_offset(200, 'right', join_style=2, mitre_limit=5)
        ke200 = list(poly_line_offset.convex_hull.boundary.coords)
        msp.add_lwpolyline(np.append(np.array(ke200), [np.array(ke200)[0]], axis=0), dxfattribs={'layer': 'KE200'})

    strgage1 = strgauge(rad, Ea, RSoutline, ctrthr, endthr)
    strgage = []

######################################################################################################################################################################################################################
# Draw Extreme Right Horizontal Inside Curve
    # Draw RS Outline (rad>0)
    if rad > 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tolerance
        EaRCtol = trkEaPl(stdgauge).dot(EaRC)
        msp.add_lwpolyline([trkEaPl(stdgauge).dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tolerance & rotational

        msp.add_lwpolyline([RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tolerance & rotational & RS Later movement

        msp.add_lwpolyline([RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})
    
    # Draw RS on Ea (rad<0), mirroring the other side
    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})


        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkEaPl(stdgauge).dot(EaRC))

        msp.add_lwpolyline([mirror().dot(trkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [mirror().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([mirror().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left Horizontal Outside Curve
    if rad > 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkEaNg(stdgauge).dot(EaRC)
        msp.add_lwpolyline([trkEaNg(stdgauge).dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline([RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})
#mirroring the other side
    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline([mirror().dot((EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkEaNg(stdgauge).dot(EaRC))

        msp.add_lwpolyline([mirror().dot(trkEaNg(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [mirror().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([mirror().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve
    if rad > 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC))))
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(
            trklatPl(track, rad).dot(
                RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left UP VERTICAL Horizontal Outside Curve

    if rad > 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot((EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC))))

        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(
            trklatNg(track, rad).dot(
                RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Right DOWN VERTICAL Horizontal Inside Curve
    if rad > 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC))

        msp.add_lwpolyline([trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([mirror().dot(
            trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))
                            for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left DOWN VERTICAL Horizontal Outside Curve

    if rad > 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC))

        msp.add_lwpolyline([trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC)))

        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(
            trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))
                            for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ## Setup Convex Hull
    # Draw Extreme Right Horizontal Inside Curve

    if rad > 0 or Ea == 0:
        EaRCtol = trkEaPl(stdgauge).dot(EaRC)
        xi1 = [[RS[n]] for n in range(len(RS) - 1)]
        xi2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xi3 = [[trkEaPl(stdgauge).dot(EaRS[n])] for n in range(len(RS) - 1)]
        xi4 = [[RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xi5 = [[RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xi6 = [[trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
            range(len(RS) - 1)]
        xi7 = [[ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkEaPl(stdgauge).dot(EaRC))
        i1 = [[RS[n]] for n in range(len(RS) - 1)]
        i2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        i3 = [[mirror().dot(trkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        i4 = [[mirror().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        i5 = [[mirror().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n
              in range(len(RS) - 1)]
        i6 = [[mirror().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        i7 = [[mirror().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        i = i1 + i2 + i3 + i4 + i5 + i6 + i7 + xi1 + xi2 + xi3 + xi4 + xi5 + xi6 + xi7
    else:
        try:
            i = i1 + i2 + i3 + i4 + i5 + i6 + i7
        except:
            pass
        try:
            i = xi1 + xi2 + xi3 + xi4 + xi5 + xi6 + xi7
        except:
            pass
    ai = np.array([i[n][0][:].tolist() for n in range(len(i))])

    # Draw Extreme Left Horizontal Outside Curve
    if rad > 0 or Ea == 0:
        EaRCtol = trkEaNg(stdgauge).dot(EaRC)
        xa1 = [[RS[n]] for n in range(len(RS) - 1)]
        xa2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xa3 = [[trkEaNg(stdgauge).dot(EaRS[n])] for n in range(len(RS) - 1)]
        xa4 = [[RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xa5 = [[RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xa6 = [[trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
            range(len(RS) - 1)]
        xa7 = [[endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkEaNg(stdgauge).dot(EaRC))
        a1 = [[RS[n]] for n in range(len(RS) - 1)]
        a2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        a3 = [[mirror().dot(trkEaNg(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        a4 = [[mirror().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        a5 = [[mirror().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n
              in range(len(RS) - 1)]
        a6 = [[mirror().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        a7 = [[mirror().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        a = a1 + a2 + a3 + a4 + a5 + a6 + a7 + xa1 + xa2 + xa3 + xa4 + xa5 + xa6 + xa7
    else:
        try:
            a = a1 + a2 + a3 + a4 + a5 + a6 + a7
        except:
            pass
        try:
            a = xa1 + xa2 + xa3 + xa4 + xa5 + xa6 + xa7
        except:
            pass
    aa = np.array([a[n][0][:].tolist() for n in range(len(a))])

    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve

    if rad > 0 or Ea == 0:
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC)))
        xb1 = [[RS[n]] for n in range(len(RS) - 1)]
        xb2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xb3 = [[trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xb4 = [[trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
               range(len(RS) - 1)]
        xb5 = [[trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n
            in range(len(RS) - 1)]
        xb6 = [[trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        xb7 = [[trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRC))))
        b1 = [[RS[n]] for n in range(len(RS) - 1)]
        b2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        b3 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        b4 = [
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        b5 = [[mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n
            in range(len(RS) - 1)]
        b6 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
        b7 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        b = b1 + b2 + b3 + b4 + b5 + b6 + b7 + xb1 + xb2 + xb3 + xb4 + xb5 + xb6 + xb7
    else:
        try:
            b = b1 + b2 + b3 + b4 + b5 + b6 + b7
        except:
            pass
        try:
            b = xb1 + xb2 + xb3 + xb4 + xb5 + xb6 + xb7
        except:
            pass
    bb = np.array([b[n][0][:].tolist() for n in range(len(b))])

    # Draw Extreme Left UP VERTICAL Horizontal Outside Curve

    if rad > 0 or Ea == 0:
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC)))
        xc1 = [[RS[n]] for n in range(len(RS) - 1)]
        xc2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xc3 = [[trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xc4 = [[trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
               range(len(RS) - 1)]
        xc5 = [[trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n
            in range(len(RS) - 1)]
        xc6 = [[trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        xc7 = [[trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRC))))
        c1 = [[RS[n]] for n in range(len(RS) - 1)]
        c2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        c3 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        c4 = [
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        c5 = [[mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for
            n in range(len(RS) - 1)]
        c6 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
        c7 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))))] for n in
            range(len(RS) - 1)]

    if Ea == 0:
        c = c1 + c2 + c3 + c4 + c5 + c6 + c7 + xc1 + xc2 + xc3 + xc4 + xc5 + xc6 + xc7
    else:
        try:
            c = c1 + c2 + c3 + c4 + c5 + c6 + c7
        except:
            pass
        try:
            c = xc1 + xc2 + xc3 + xc4 + xc5 + xc6 + xc7
        except:
            pass
    cc = np.array([c[n][0][:].tolist() for n in range(len(c))])

    # Draw Extreme Right DOWN VERTICAL Horizontal Inside Curve

    if rad > 0 or Ea == 0:
        EaRCtol = trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC))
        xd1 = [[RS[n]] for n in range(len(RS) - 1)]
        xd2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xd3 = [[trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xd4 = [[trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xd5 = [
            [trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))]
            for n in range(len(RS) - 1)]
        xd6 = [[trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        xd7 = [[trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRC)))
        d1 = [[RS[n]] for n in range(len(RS) - 1)]
        d2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        d3 = [[mirror().dot(trkvertNg(track).dot(trkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        d4 = [[mirror().dot(trkvertNg(track).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        d5 = [[mirror().dot(
            trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        d6 = [[mirror().dot(trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        d7 = [[mirror().dot(trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(trkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        d = d1 + d2 + d3 + d4 + d5 + d6 + d7 + xd1 + xd2 + xd3 + xd4 + xd5 + xd6 + xd7
    else:
        try:
            d = d1 + d2 + d3 + d4 + d5 + d6 + d7
        except:
            pass
        try:
            d = xd1 + xd2 + xd3 + xd4 + xd5 + xd6 + xd7
        except:
            pass
    dd = np.array([d[n][0][:].tolist() for n in range(len(d))])

    # Draw Extreme Left DOWN VERTICAL Horizontal Outside Curve

    if rad > 0 or Ea == 0:
        EaRCtol = trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC))
        xe1 = [[RS[n]] for n in range(len(RS) - 1)]
        xe2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xe3 = [[trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xe4 = [[trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xe5 = [
            [trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))]
            for n in range(len(RS) - 1)]
        xe6 = [[trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        xe7 = [[trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRC)))
        e1 = [[RS[n]] for n in range(len(RS) - 1)]
        e2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        e3 = [[mirror().dot(trkvertNg(track).dot(trkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        e4 = [[mirror().dot(trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        e5 = [[mirror().dot(
            trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        e6 = [[mirror().dot(trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        e7 = [[mirror().dot(trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(trkEaNg(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        e = e1 + e2 + e3 + e4 + e5 + e6 + e7 + xe1 + xe2 + xe3 + xe4 + xe5 + xe6 + xe7
    else:
        try:
            e = e1 + e2 + e3 + e4 + e5 + e6 + e7
        except:
            pass
        try:
            e = xe1 + xe2 + xe3 + xe4 + xe5 + xe6 + xe7
        except:
            pass
    ee = np.array([e[n][0][:].tolist() for n in range(len(e))])

    ii = np.concatenate((ai, aa, bb, cc, dd, ee))

    if rad!=0:

        conai = np.array([ii[n].tolist() for n in range(len(ii))])
        ConHu = ConvexHull(np.delete(conai, 2, 1)).vertices
        OutLConHu = ii[ConHu]
        msp.add_lwpolyline(np.append(np.array(OutLConHu), [np.array(OutLConHu)[0]], axis=0), dxfattribs={'layer': 'KE'})

        h = []

        for n in range(len(ConHu)):
            h.append([ii[ConHu, 0][n], ii[ConHu, 1][n]])
        poly_line = LinearRing(h)
        poly_line_offset = poly_line.parallel_offset(200, 'right', join_style=2, mitre_limit=5)
        ke200 = list(poly_line_offset.convex_hull.boundary.coords)
        msp.add_lwpolyline(np.append(np.array(ke200), [np.array(ke200)[0]], axis=0), dxfattribs={'layer': 'KE200'})

    strgage1 = strgauge(rad, Ea, RSoutline, ctrthr, endthr)
    strgage = []

    if rad > 0 or rad == 0:
        strgage = strgauge(rad, Ea, RSoutline, ctrthr, endthr)
    if rad < 0:
        for n in range(len(strgage1) - 1):
            strgage.append(mirror().dot(strgauge(rad, Ea, RSoutline, ctrthr, endthr)[n]))

    msp.add_lwpolyline(np.append(np.array(strgage), [np.array(strgage)[0]], axis=0),
                       dxfattribs={'layer': 'Structural Gauge'})
    msp.add_text(str(location),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1000), align='MIDDLE_LEFT')

    msp.add_text(str(RSoutline),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1250), align='MIDDLE_LEFT')

    msp.add_text("Cant: " + str(Ea) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1500), align='MIDDLE_LEFT')

    msp.add_text("Curve Radius: " + str(rad / 1000) + "m",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1750), align='MIDDLE_LEFT')

    msp.add_text("Centre Throw: " + str(round(ctrthr, 2)) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2000), align='MIDDLE_LEFT')

    msp.add_text("End Throw: " + str(round(endthr, 2)) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2250), align='MIDDLE_LEFT')

    msp.add_text("Track Tolerances",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2750), align='MIDDLE_LEFT')

    msp.add_text("Track Type: " + str(track),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3000), align='MIDDLE_LEFT')

    msp.add_text("Rail Wear: " + str(15) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3250), align='MIDDLE_LEFT')

    msp.add_text("Lateral Alignment: " + str(trklatPl(track, rad)[0][2] - 15) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3500), align='MIDDLE_LEFT')
    if track == 'SM_BT':
        msp.add_text("Superelevation: " + str(10) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3750), align='MIDDLE_LEFT')
    if track == 'SM_CST':
        msp.add_text("Superelevation: " + str(8) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -3750), align='MIDDLE_LEFT')

    msp.add_text("Vertical Positive: " + str(trkvertPl(track)[1][2]) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4000), align='MIDDLE_LEFT')

    msp.add_text("Vertical Negative: " + str(-trkvertNg(track)[1][2]) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4250), align='MIDDLE_LEFT')

    msp.add_text("Rolling Stock Tolerances",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4750), align='MIDDLE_LEFT')

    msp.add_text("Lateral: " + str(44) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5000), align='MIDDLE_LEFT')

    msp.add_text("Vertical: " + str(50) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5250), align='MIDDLE_LEFT')

    msp.add_text("Roll Body Only: " + str(2) + " degrees",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5500), align='MIDDLE_LEFT')

    msp.add_line((0, 0), (0, 6000))

    doc.saveas(location + '.dxf')

########################## SYDNEY METRO #####################################################
########################## ARTC CLEARANCES ##########################

def ARTCKEenve(Ea, stdgauge, rad, track, RSoutline, location):
    # Setup dxf model space and process rolling stock outline

    def EaMax(Ea, stdgauge):
        a = -np.arctan((Ea / (stdgauge * 2)))
        x = stdgauge
        y = 0
        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx


    def RSTolLatPl(Ea, stdgauge):
        tol = 60
        x = tol
        y = 0

        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSTolLatNg(Ea, stdgauge):
        tol = -60
        x = tol
        y = 0

        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSTolVert():
        tol = 50
        x = 0
        y = tol
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

        return mx


    def RSRotCCW(RollCen):
        a = np.deg2rad(2)
        x = RollCen[0]
        y = RollCen[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])

        return mx


    def RSRotCW(RollCen):
        a = np.deg2rad(-2)
        x = RollCen[0]
        y = RollCen[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx


    def trklatPl(track, rad):
        if rad == 0:
            x = 5 + 20 + 50
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

        if rad > 300000:
            x = 25 + 20 + 50
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

        if rad < 300000:
            x = 25 + 20 + 75
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

    def trklatNg(track, rad):
        if rad == 0:
            x = -(5 + 20 + 50)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

        if rad > 300000:
            x = -(5 + 20 + 50)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

        if rad < 300000:
            x = -(5 + 20 + 75 + 15)
            y = 0
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx

    def trkvertPl(track):
        x = 0
        y = 100
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        return mx


    def trkvertNg(track):
            x = 0
            y = -100
            mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
            return mx


    def INtrkEaPl(stdgauge):
        Ea = 30
        a = -np.arcsin((Ea / (stdgauge * 2)))
        x = stdgauge
        y = 0
        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx


    def INtrkEaNg(stdgauge):
        Ea = -30
        a = -np.arcsin((Ea / (stdgauge * 2)))
        x = stdgauge
        y = 0
        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])
        return mx

    def OUTtrkEaPl(stdgauge):
        Ea = 30
        a = np.arcsin((Ea / (stdgauge * 2)))
        c = EaMax(Ea, stdgauge).dot(np.array([-stdgauge, 0, 1]))

        x = c[0]
        y = c[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])

        return mx


    def OUTtrkEaNg(stdgauge):
        Ea = -30
        a = np.arcsin((Ea / (stdgauge * 2)))
        c = EaMax(Ea, stdgauge).dot(np.array([-stdgauge, 0, 1]))

        x = c[0]
        y = c[1]

        mx = np.array([[np.cos(a), -np.sin(a), -x * np.cos(a) + y * np.sin(a) + x],
                       [np.sin(a), np.cos(a), -x * np.sin(a) - y * np.cos(a) + y], [0, 0, 1]])

        return mx
    def ctrthrow(ctrthr):
        x = ctrthr + 5
        y = 0
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        return mx


    def endthrow(endthr):
        x = -endthr - 5
        y = 0
        mx = np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])
        return mx


    def mirror():
        mx = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
        return mx


    def strgauge(rad, Ea, RSoutline, ctrthr, endthr):
        from re import search
        Z = 3800
        B = 4670 + 1.2*Ea
        if rad == 0:
            M = 0
        if rad != 0:
            M = 42000/(np.abs(rad)/1000)

        strgage = [[0, 0, 1],
                   [2060 + M + Ea * Z / 1435, 0, 1],
                   [2060 + M + Ea * Z / 1435, Z, 1],
                   [1525 + M + Ea * B / 1435, B, 1],
                   [0, B, 1],
                   [-1525 - M + Ea * B / 1435, B, 1],
                   [-2060 - M + Ea * Z / 1435, Z, 1],
                   [-2060 - M + Ea * Z / 1435, 0, 1],
                   [0, 0, 1]]
        return strgage

    RollCen = np.array([0, 610, 1])
    doc = ezdxf.new()
    doc.units = units.MM
    RS_CS = pd.read_csv(RSoutline)
    RS_CS['X'] = RS_CS['X'].astype('float')
    RS_CS['Y'] = RS_CS['Y'].astype('float')
    bogctr = RS_CS['X'][0]
    bodovr = RS_CS['X'][1]
    vecwid = RS_CS['X'][2]

    doc = ezdxf.new(setup=True)
    msp = doc.modelspace()  # add new entities to the modelspace
    # dxf layers list

    doc.layers.new(name='Rolling Stock OL', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='Rolling Stock on Ea', dxfattribs={'linetype': 'CONTINUOUS', 'color': 1})

    doc.layers.new(name='RS&Ea&Eatol', dxfattribs={'linetype': 'CONTINUOUS', 'color': 2})

    doc.layers.new(name='RS&Ea&Eatol&roll', dxfattribs={'linetype': 'CONTINUOUS', 'color': 3})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol', dxfattribs={'linetype': 'CONTINUOUS', 'color': 4})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol&trkalign', dxfattribs={'linetype': 'CONTINUOUS', 'color': 5})

    doc.layers.new(name='RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr', dxfattribs={'linetype': 'CONTINUOUS', 'color': 6})

    doc.layers.new(name='KE200', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='Structural Gauge', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    doc.layers.new(name='KE', dxfattribs={'linetype': 'CONTINUOUS', 'color': 0})

    # Data setup

    RS_R = np.transpose([np.array(RS_CS.iloc[4:RS_CS.shape[0], 1]), np.array(RS_CS.iloc[4:RS_CS.shape[0], 2]),
                         np.ones(RS_CS.shape[0] - 4)])
    RS_L = np.transpose([np.array(-RS_CS.iloc[4:RS_CS.shape[0], 1]), np.array(RS_CS.iloc[4:RS_CS.shape[0], 2]),
                         np.ones(RS_CS.shape[0] - 4)])
    RS = np.append(RS_R, RS_L[::-1], axis=0)

    if rad == 0:
        ctrthr = 0
    if rad > 0 or rad < 0:
        ctrthr = (bogctr ** 2) / (8 * np.abs(rad))
    if rad == 0:
        endthr = 0
    if rad > 0 or rad < 0:
        endthr = ((((bogctr + 2 * bodovr) ** 2) / (8 * np.abs(rad) + 4 * vecwid))) - ctrthr
    # EA applied to RS points
    EaRS = [np.array(EaMax(Ea, stdgauge).dot(RS[n])) for n in range(len(RS))]

    EaRC = np.array(EaMax(Ea, stdgauge).dot(RollCen))

    EaRCzero = np.array(EaMax(Ea, stdgauge).dot([0, 0, 1]))
    # Draw Extreme Right Horizontal Inside Curve

    # Draw RS Outline
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = INtrkEaPl(stdgauge).dot(EaRC)
        msp.add_lwpolyline([INtrkEaPl(stdgauge).dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline([RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline([mirror().dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(INtrkEaPl(stdgauge).dot(EaRC))

        msp.add_lwpolyline([mirror().dot(INtrkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [mirror().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([mirror().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left Horizontal Outside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline(EaRS, dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = OUTtrkEaPl(stdgauge).dot(EaRC)
        msp.add_lwpolyline([OUTtrkEaNg(stdgauge).dot(EaRS[n]) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline([RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea

        msp.add_lwpolyline([mirror().dot((EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(OUTtrkEaPl(stdgauge).dot(EaRC))

        msp.add_lwpolyline([mirror().dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [mirror().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([mirror().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(EaRS[n])) for n in range(len(EaRS))], dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(EaRS[n]))) for n in range(len(EaRS))], dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRC))))
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(
            trklatPl(track, rad).dot(
                RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left UP VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(EaRS[n])) for n in range(len(EaRS))], dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw

        msp.add_lwpolyline([trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(EaRS[n]))) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRC))))

        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))) for n
                            in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([mirror().dot(trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(
            trklatNg(track, rad).dot(
                RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    # Draw Extreme Right DOWN VERTICAL Horizontal Inside Curve
    ########################################################################################################################################################################################################################################################################################################################################################
    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([trkvertNg(track).dot(EaRS[n]) for n in range(len(EaRS))], dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRC))

        msp.add_lwpolyline([trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRC)))
        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll

        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat

        msp.add_lwpolyline([mirror().dot(
            trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))))
                            for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align

        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & centre throw

        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ########################################################################################################################################################################################################################################################################################################################################################
    # Draw Extreme Left DOWN VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([trkvertNg(track).dot(EaRS[n]) for n in range(len(EaRS))], dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRC))

        msp.add_lwpolyline([trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline(
            [trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))
             for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    if rad < 0 or Ea == 0:
        msp.add_lwpolyline(RS, dxfattribs={'layer': 'Rolling Stock OL'})

        # Draw RS on Ea
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(EaRS[n])) for n in range(len(EaRS))],
                           dxfattribs={'layer': 'Rolling Stock on Ea'})

        # KE envelope
        # KE Superelevation Tol
        EaRCtol = mirror().dot(trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRC)))

        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))) for n in range(len(EaRS))],
            dxfattribs={'layer': 'RS&Ea&Eatol'})

        # KE Superelevation Tol & roll
        msp.add_lwpolyline(
            [mirror().dot(trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))) for n in
             range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll'})

        # KE Superelevation Tol & roll & RS Lat
        msp.add_lwpolyline([mirror().dot(
            trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))
                            for n in range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol'})

        # KE Superelevation Tol & roll & RS Lat & Track align
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign'})

        # KE Superelevation Tol & roll & RS Lat & Track align & end throw
        msp.add_lwpolyline([mirror().dot(trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))) for n in
                            range(len(EaRS))], dxfattribs={'layer': 'RS&Ea&Eatol&roll&RSlatol&trkalign&ctrthr'})

    ## Setup Convex Hull
    # Draw Extreme Right Horizontal Inside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = INtrkEaPl(stdgauge).dot(EaRC)
        xi1 = [[RS[n]] for n in range(len(RS) - 1)]
        xi2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xi3 = [[INtrkEaPl(stdgauge).dot(EaRS[n])] for n in range(len(RS) - 1)]
        xi4 = [[RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xi5 = [[RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xi6 = [[trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))] for n in
            range(len(RS) - 1)]
        xi7 = [[ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(INtrkEaPl(stdgauge).dot(EaRC))
        i1 = [[RS[n]] for n in range(len(RS) - 1)]
        i2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        i3 = [[mirror().dot(INtrkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        i4 = [[mirror().dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        i5 = [[mirror().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))] for n
              in range(len(RS) - 1)]
        i6 = [[mirror().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        i7 = [[mirror().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        i = i1 + i2 + i3 + i4 + i5 + i6 + i7 + xi1 + xi2 + xi3 + xi4 + xi5 + xi6 + xi7
    else:
        try:
            i = i1 + i2 + i3 + i4 + i5 + i6 + i7
        except:
            pass
        try:
            i = xi1 + xi2 + xi3 + xi4 + xi5 + xi6 + xi7
        except:
            pass
    ai = np.array([i[n][0][:].tolist() for n in range(len(i))])

    # Draw Extreme Left Horizontal Outside Curve
    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = OUTtrkEaPl(stdgauge).dot(EaRC)
        xa1 = [[RS[n]] for n in range(len(RS) - 1)]
        xa2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xa3 = [[OUTtrkEaPl(stdgauge).dot(EaRS[n])] for n in range(len(RS) - 1)]
        xa4 = [[RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xa5 = [[RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xa6 = [[trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))] for n in
            range(len(RS) - 1)]
        xa7 = [[endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(OUTtrkEaPl(stdgauge).dot(EaRC))
        a1 = [[RS[n]] for n in range(len(RS) - 1)]
        a2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        a3 = [[mirror().dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        a4 = [[mirror().dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        a5 = [[mirror().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))] for n
              in range(len(RS) - 1)]
        a6 = [[mirror().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        a7 = [[mirror().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        a = a1 + a2 + a3 + a4 + a5 + a6 + a7 + xa1 + xa2 + xa3 + xa4 + xa5 + xa6 + xa7
    else:
        try:
            a = a1 + a2 + a3 + a4 + a5 + a6 + a7
        except:
            pass
        try:
            a = xa1 + xa2 + xa3 + xa4 + xa5 + xa6 + xa7
        except:
            pass
    aa = np.array([a[n][0][:].tolist() for n in range(len(a))])

    # Draw Extreme Right UP VERTICAL Horizontal Inside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRC)))
        xb1 = [[RS[n]] for n in range(len(RS) - 1)]
        xb2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xb3 = [[trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xb4 = [[trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))] for n in
               range(len(RS) - 1)]
        xb5 = [[trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))] for n
            in range(len(RS) - 1)]
        xb6 = [[trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        xb7 = [[trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRC))))
        b1 = [[RS[n]] for n in range(len(RS) - 1)]
        b2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        b3 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        b4 = [
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        b5 = [[mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))))] for n
            in range(len(RS) - 1)]
        b6 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
        b7 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaPl(stdgauge).dot(EaRS[n]))))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        b = b1 + b2 + b3 + b4 + b5 + b6 + b7 + xb1 + xb2 + xb3 + xb4 + xb5 + xb6 + xb7
    else:
        try:
            b = b1 + b2 + b3 + b4 + b5 + b6 + b7
        except:
            pass
        try:
            b = xb1 + xb2 + xb3 + xb4 + xb5 + xb6 + xb7
        except:
            pass
    bb = np.array([b[n][0][:].tolist() for n in range(len(b))])

    # Draw Extreme Left UP VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRC)))
        xc1 = [[RS[n]] for n in range(len(RS) - 1)]
        xc2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xc3 = [[trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xc4 = [[trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))] for n in
               range(len(RS) - 1)]
        xc5 = [[trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))] for n
            in range(len(RS) - 1)]
        xc6 = [[trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        xc7 = [[trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRC))))
        c1 = [[RS[n]] for n in range(len(RS) - 1)]
        c2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        c3 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        c4 = [
            [mirror().dot(trkvertPl(track).dot(RSTolVert().dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        c5 = [[mirror().dot(trkvertPl(track).dot(
            RSTolVert().dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))] for
            n in range(len(RS) - 1)]
        c6 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
        c7 = [[mirror().dot(trkvertPl(track).dot(RSTolVert().dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))))] for n in
            range(len(RS) - 1)]

    if Ea == 0:
        c = c1 + c2 + c3 + c4 + c5 + c6 + c7 + xc1 + xc2 + xc3 + xc4 + xc5 + xc6 + xc7
    else:
        try:
            c = c1 + c2 + c3 + c4 + c5 + c6 + c7
        except:
            pass
        try:
            c = xc1 + xc2 + xc3 + xc4 + xc5 + xc6 + xc7
        except:
            pass
    cc = np.array([c[n][0][:].tolist() for n in range(len(c))])

    # Draw Extreme Right DOWN VERTICAL Horizontal Inside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRC))
        xd1 = [[RS[n]] for n in range(len(RS) - 1)]
        xd2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xd3 = [[trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xd4 = [[trkvertNg(track).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        xd5 = [
            [trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))))]
            for n in range(len(RS) - 1)]
        xd6 = [[trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        xd7 = [[trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRC)))
        d1 = [[RS[n]] for n in range(len(RS) - 1)]
        d2 = [[mirror().dot(EaRS[n])] for n in range(len(RS) - 1)]
        d3 = [[mirror().dot(trkvertNg(track).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        d4 = [[mirror().dot(trkvertNg(track).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        d5 = [[mirror().dot(
            trkvertNg(track).dot(RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        d6 = [[mirror().dot(trkvertNg(track).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        d7 = [[mirror().dot(trkvertNg(track).dot(ctrthrow(ctrthr).dot(trklatPl(track, rad).dot(
            RSTolLatPl(Ea, stdgauge).dot(RSRotCW(EaRCtol).dot(INtrkEaNg(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        d = d1 + d2 + d3 + d4 + d5 + d6 + d7 + xd1 + xd2 + xd3 + xd4 + xd5 + xd6 + xd7
    else:
        try:
            d = d1 + d2 + d3 + d4 + d5 + d6 + d7
        except:
            pass
        try:
            d = xd1 + xd2 + xd3 + xd4 + xd5 + xd6 + xd7
        except:
            pass
    dd = np.array([d[n][0][:].tolist() for n in range(len(d))])

    # Draw Extreme Left DOWN VERTICAL Horizontal Outside Curve

    if rad > 0 or rad == 0 or Ea == 0:
        EaRCtol = trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRC))
        xe1 = [[RS[n]] for n in range(len(RS) - 1)]
        xe2 = [[EaRS[n]] for n in range(len(RS) - 1)]
        xe3 = [[trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))] for n in range(len(RS) - 1)]
        xe4 = [[trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))] for n in
               range(len(RS) - 1)]
        xe5 = [
            [trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))]
            for n in range(len(RS) - 1)]
        xe6 = [[trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))] for n in
            range(len(RS) - 1)]
        xe7 = [[trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]

    if rad < 0 or Ea == 0:
        EaRCtol = mirror().dot(trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRC)))
        e1 = [[RS[n]] for n in range(len(RS) - 1)]
        e2 = [[mirror().dot((EaRS[n]))] for n in range(len(RS) - 1)]
        e3 = [[mirror().dot(trkvertNg(track).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))] for n in range(len(RS) - 1)]
        e4 = [[mirror().dot(trkvertNg(track).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))] for n in
              range(len(RS) - 1)]
        e5 = [[mirror().dot(
            trkvertNg(track).dot(RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))]
            for n in range(len(RS) - 1)]
        e6 = [[mirror().dot(trkvertNg(track).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n]))))))] for n in
            range(len(RS) - 1)]
        e7 = [[mirror().dot(trkvertNg(track).dot(endthrow(endthr).dot(trklatNg(track, rad).dot(
            RSTolLatNg(Ea, stdgauge).dot(RSRotCCW(EaRCtol).dot(OUTtrkEaPl(stdgauge).dot(EaRS[n])))))))] for n in
            range(len(RS) - 1)]
    if Ea == 0:
        e = e1 + e2 + e3 + e4 + e5 + e6 + e7 + xe1 + xe2 + xe3 + xe4 + xe5 + xe6 + xe7
    else:
        try:
            e = e1 + e2 + e3 + e4 + e5 + e6 + e7
        except:
            pass
        try:
            e = xe1 + xe2 + xe3 + xe4 + xe5 + xe6 + xe7
        except:
            pass
    ee = np.array([e[n][0][:].tolist() for n in range(len(e))])
    ii = np.concatenate((ai, aa, bb, cc, dd, ee))

    conai = np.array([ii[n].tolist() for n in range(len(ii))])
    ConHu = ConvexHull(np.delete(conai, 2, 1)).vertices
    OutLConHu = ii[ConHu]
    msp.add_lwpolyline(np.append(np.array(OutLConHu), [np.array(OutLConHu)[0]], axis=0), dxfattribs={'layer': 'KE'})

    h = []
    for n in range(len(ConHu)):
        h.append([ii[ConHu, 0][n], ii[ConHu, 1][n]])
    poly_line = LinearRing(h)
    poly_line_offset = poly_line.parallel_offset(200, 'right', join_style=2, mitre_limit=5)
    ke200 = list(poly_line_offset.convex_hull.boundary.coords)
    msp.add_lwpolyline(np.append(np.array(ke200), [np.array(ke200)[0]], axis=0), dxfattribs={'layer': 'KE200'})

    strgage1 = strgauge(rad, Ea, RSoutline, ctrthr, endthr)
    strgage = []

    if rad > 0 or rad == 0:
        strgage = strgauge(rad, Ea, RSoutline, ctrthr, endthr)
    if rad < 0:
        for n in range(len(strgage1) - 1):
            strgage.append(mirror().dot(strgauge(rad, Ea, RSoutline, ctrthr, endthr)[n]))

    msp.add_lwpolyline(np.append(np.array(strgage), [np.array(strgage)[0]], axis=0),
                       dxfattribs={'layer': 'Structural Gauge'})
    msp.add_text(str(location),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1000), align='MIDDLE_LEFT')

    msp.add_text(str(RSoutline),
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1250), align='MIDDLE_LEFT')

    msp.add_text("Cant: " + str(Ea) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1500), align='MIDDLE_LEFT')

    msp.add_text("Curve Radius: " + str(rad / 1000) + "m",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -1750), align='MIDDLE_LEFT')

    msp.add_text("Centre Throw: " + str(round(ctrthr, 2)) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2000), align='MIDDLE_LEFT')

    msp.add_text("End Throw: " + str(round(endthr, 2)) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2250), align='MIDDLE_LEFT')

    msp.add_text("Track Tolerances",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -2750), align='MIDDLE_LEFT')

    if np.abs(rad) < 300000:
        msp.add_text("Gauge (from 1435 mm): 20mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3000), align='MIDDLE_LEFT')
    else:
        msp.add_text("Gauge (from 1435 mm): " + str(25) + "mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3000), align='MIDDLE_LEFT')

    if np.abs(rad) == 0:
        msp.add_text("Rail Wear: " + str(5) + "mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3250), align='MIDDLE_LEFT')
    else:
        msp.add_text("Rail Wear: +25mm, -5 mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3250), align='MIDDLE_LEFT')

    if np.abs(rad) < 300000:
        msp.add_text("Gauge widening: +0mm, -15mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3500), align='MIDDLE_LEFT')
    else:
        msp.add_text("Gauge widening: " + str(0) + "mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3500), align='MIDDLE_LEFT')

    if np.abs(rad) < 300000:
        msp.add_text("Track alignment (from design): " + str(75) + "mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3750), align='MIDDLE_LEFT')
    else:
        msp.add_text("Track alignment (from design): " + str(50) + "mm",
                     dxfattribs={
                         'style': 'LiberationSerif',
                         'height': 150}
                     ).set_pos((-1000, -3750), align='MIDDLE_LEFT')

    msp.add_text("Cross-level (from design): " + str(30) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4000), align='MIDDLE_LEFT')

    msp.add_text("Rail Level: " + str(100) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4250), align='MIDDLE_LEFT')

    msp.add_text("Rolling Stock Tolerances",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -4750), align='MIDDLE_LEFT')

    msp.add_text("Lateral: " + str(40) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5000), align='MIDDLE_LEFT')

    msp.add_text("Bounce: " + str(50) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5250), align='MIDDLE_LEFT')

    msp.add_text("Body Roll: " + str(2) + " degrees",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5500), align='MIDDLE_LEFT')

    msp.add_text("Wheel clearance (worn wheel to new rail): " + str(20) + "mm",
                 dxfattribs={
                     'style': 'LiberationSerif',
                     'height': 150}
                 ).set_pos((-1000, -5750), align='MIDDLE_LEFT')
    msp.add_line((0, 0), (0, 6000))

    doc.saveas(location + '.dxf')

########################## ARTC CLEARANCES ##########################

# [Setup] inputs all info from SETUP.csv
setup = pd.read_csv('SETUP.csv')

Ea = []
rad = []
track = []
RSoutline = []
location = []
standard = []

stdgauge = 717.5  # half of track gauge


for n in range(len(setup)):
    location.append(setup['Location'][n])
    rad.append(float(setup['CurveRadius(m)'][n]) * 1000)
    Ea.append(float(setup['Superelevation(mm)'][n]))
    track.append(str(setup['Track'][n]))
    RSoutline.append(str(setup['RollingStock'][n]))
    standard.append(str(setup['Standard'][n]))

for n in range(len(setup)):
    if standard[n] == 'ASA':
        ASAKEenve(Ea[n], stdgauge, rad[n], track[n], RSoutline[n], location[n])
    if standard[n] == 'ARTC':
        ARTCKEenve(Ea[n], stdgauge, rad[n], track[n], RSoutline[n], location[n])
    if standard[n] == 'SM':
        SMKEenve(Ea[n], stdgauge, rad[n], track[n], RSoutline[n], location[n])