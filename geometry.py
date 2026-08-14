# 定义关键点
import configparser
import copy
import logging
import math
import os
import sys

import gmsh
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet2d
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeRevol, BRepPrimAPI_MakeCylinder
from OCC.Core.GC import GC_MakeArcOfEllipse, GC_MakeSegment
from OCC.Core.Geom2dAPI import Geom2dAPI_InterCurveCurve
from OCC.Core.Quantity import Quantity_Color, Quantity_NOC_WHITE
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.TopAbs import TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods_Vertex, TopoDS_Shape, TopoDS_Compound
from OCC.Core._Quantity import Quantity_TOC_RGB
from OCC.Core.gp import gp_Elips, gp_Pln, gp_Cylinder, gp_Ax3
from OCC.Core import TopExp, TopoDS, GeomAPI
from OCC.Core.AIS import AIS_Line
from OCC.Core.BRep import BRep_Tool, BRep_Builder
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_Transform, BRepBuilderAPI_MakeWire, \
    BRepBuilderAPI_MakeFace
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.GccAna import GccAna_Lin2dTanObl, GccAna_Circ2dTanOnRad, GccAna_Lin2dTanPar, GccAna_Circ2d2TanRad, \
    GccAna_Lin2d2Tan
from OCC.Core.GccEnt import GccEnt_QualifiedCirc, GccEnt_Position, GccEnt_QualifiedLin
from OCC.Core.Geom import Geom_Line, Geom_Circle
from OCC.Core.GeomAPI import GeomAPI_ExtremaCurveCurve
from OCC.Core.TopoDS import topods_Edge
from OCC.Core.gp import gp_Pnt, gp_Ax2, gp_Trsf, gp_Ax1, gp_Dir, gp_Vec, gp_Circ, gp_Lin2d, \
    gp_Circ2d, gp_Ax2d, gp_Pnt2d, gp_Dir2d, gp_Lin
from OCC.Display.SimpleGui import init_display
from trimesh.exchange.stl import export_stl

import SRMMesh
import properties
from SRMOCC.exportOBJ import export_obj
from SRMOCC.exportSTEP import exportSTEP
from SRMOCC.exportSTL import export_to_stl
from SRMOCC.intersection import get_non_adjacent_edge_pairs, edges_have_intersection
from SRMOCC.surface import Surface, compare_surface
from SRMOCC.transform import rotate_solid_around_x


def display(solid):
    viewer, start_display, add_menu, add_function_to_menu = init_display()
    viewer.View.SetBackgroundColor(
        Quantity_Color(1.0, 1.0, 1.0, Quantity_TOC_RGB)
    )
    viewer.DisplayShape(solid, update=True)
    start_display()

def outer(a, b, L, N, scale=1):
    """
    Make an outer of grain. 有前后封头
    Parameters:
        a - 短半轴
        b - 长半轴
        L - 裙间距
        N - 对称数
        scale - 放缩因子
    """
    a, b, L = a * scale, b * scale, L * scale
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(a, 0, 0)
    p3 = gp_Pnt(0, b, 0)
    p4 = gp_Pnt(-a, 0, 0)
    p5 = gp_Pnt(L, b, 0)
    p6 = gp_Pnt(L + a, 0, 0)

    # 绘制椭圆
    ellipse_axis = gp_Ax2(p1, gp_Dir(0, 0, 1))
    ellipse = gp_Elips(ellipse_axis, b, a)

    # 创建四分之一椭圆弧
    arc = GC_MakeArcOfEllipse(ellipse, p2, p3, True).Value()  # 逆时针

    # 生成边界线
    edge = BRepBuilderAPI_MakeEdge(arc).Edge()

    # 进行 90° 旋转变换（绕 Z 轴）
    transformation = gp_Trsf()
    transformation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.radians(90))  # 角度转弧度
    transformed_edge = BRepBuilderAPI_Transform(edge, transformation)

    mirror_trsf = gp_Trsf()
    mirror_trsf.SetMirror(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)))  # 关于 YZ 平面
    mirrored_shape = BRepBuilderAPI_Transform(topods_Edge(transformed_edge.Shape()), mirror_trsf).Shape()

    translation_trsf = gp_Trsf()
    translation_trsf.SetTranslation(gp_Pnt(0, 0, 0), gp_Pnt(L, 0, 0))
    translated_shape = BRepBuilderAPI_Transform(mirrored_shape, translation_trsf).Shape()

    edge_p3_p5 = BRepBuilderAPI_MakeEdge(p3, p5).Edge()
    edge_p6_p4 = BRepBuilderAPI_MakeEdge(p6, p4).Edge()
    edge_ellipse = topods_Edge(transformed_edge.Shape())
    wire_builder = BRepBuilderAPI_MakeWire()

    wire_builder.Add(edge_ellipse)
    wire_builder.Add(edge_p3_p5)
    wire_builder.Add(topods_Edge(translated_shape))
    wire_builder.Add(edge_p6_p4)
    # 获取最终的 wire
    wire = wire_builder.Wire()

    # 7️⃣ **绕 X 轴旋转生成三维实体**
    axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0))  # 绕 X 轴旋转 (gp_Ax1 类型)

    # 8️⃣ **生成旋转体**
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid1 = BRepPrimAPI_MakeRevol(face, axis, math.radians(180 / N)).Shape()  # 旋转90度 (7种.5708 rad)
    solid2 = BRepPrimAPI_MakeRevol(face, axis, math.radians(-180 / N)).Shape()
    solid = BRepAlgoAPI_Fuse(solid1, solid2).Shape()

    return solid

def tube(Rin, a, L, N, scale=1):
    """
    Make an outer of grain. 没有前后封头
    Parameters:
        Rin - 内径
        a - 半径
        L - 长度
        N - 对称数
        scale - 放缩因子
    """
    a, L = a * scale, L * scale
    p1 = gp_Pnt(0, 0, Rin)
    p2 = gp_Pnt(0, 0, a)
    p3 = gp_Pnt(L, 0, a)
    p4 = gp_Pnt(L, 0, Rin)
    edge1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    edge2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    edge3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
    edge4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()
    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(edge1)
    wire_builder.Add(edge2)
    wire_builder.Add(edge3)
    wire_builder.Add(edge4)
    wire = wire_builder.Wire()

    # 7️⃣ **绕 X 轴旋转生成三维实体**
    axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0))  # 绕 X 轴旋转 (gp_Ax1 类型)

    # 8️⃣ **生成旋转体**
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid1 = BRepPrimAPI_MakeRevol(face, axis, math.radians(180 / N)).Shape()  # 旋转90度 (7种.5708 rad)
    solid2 = BRepPrimAPI_MakeRevol(face, axis, math.radians(-180 / N)).Shape()
    solid = BRepAlgoAPI_Fuse(solid1, solid2).Shape()

    return solid

def outer2(a, L, N, scale=1):
    """
    Make an outer of grain. 没有前后封头
    Parameters:
        a - 半径
        L - 长度
        N - 对称数
        scale - 放缩因子
    """
    a, L = a * scale, L * scale
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(0, 0, a)
    p3 = gp_Pnt(L, 0, a)
    p4 = gp_Pnt(L, 0, 0)
    edge1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    edge2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    edge3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
    edge4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()
    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(edge1)
    wire_builder.Add(edge2)
    wire_builder.Add(edge3)
    wire_builder.Add(edge4)
    wire = wire_builder.Wire()

    # 7️⃣ **绕 X 轴旋转生成三维实体**
    axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0))  # 绕 X 轴旋转 (gp_Ax1 类型)

    # 8️⃣ **生成旋转体**
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid1 = BRepPrimAPI_MakeRevol(face, axis, math.radians(180 / N)).Shape()  # 旋转90度 (7种.5708 rad)
    solid2 = BRepPrimAPI_MakeRevol(face, axis, math.radians(-180 / N)).Shape()
    solid = BRepAlgoAPI_Fuse(solid1, solid2).Shape()

    return solid

def inner(a, L, N, scale=1):
    """
    Make an inner of grain.
    Parameters:
        a - 内孔半径
        L - 拉伸长度
        N - 对称数
        scale - 放缩因子
    """
    a, L = a * scale, L * scale
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(0, 0, a)
    p3 = gp_Pnt(L, 0, a)
    p4 = gp_Pnt(L, 0, 0)
    edge1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    edge2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    edge3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
    edge4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()
    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(edge1)
    wire_builder.Add(edge2)
    wire_builder.Add(edge3)
    wire_builder.Add(edge4)
    wire = wire_builder.Wire()

    # 7️⃣ **绕 X 轴旋转生成三维实体**
    axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0))  # 绕 X 轴旋转 (gp_Ax1 类型)

    # 8️⃣ **生成旋转体**
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid1 = BRepPrimAPI_MakeRevol(face, axis, math.radians(180 / N)).Shape()  # 旋转90度 (7种.5708 rad)
    solid2 = BRepPrimAPI_MakeRevol(face, axis, math.radians(-180 / N)).Shape()
    solid = BRepAlgoAPI_Fuse(solid1, solid2).Shape()

    return solid

def yi(H1, H2, R1, R2, L, W, Distance, alpha1, alpha2, N, scale=1):
    flag = True
    yi = TopoDS_Shape()

    H1 = H1 * scale
    H2 = H2 * scale
    R1 = R1 * scale
    R2 = R2 * scale
    L = L * scale
    W = W * scale
    Distance = Distance * scale
    try:
        p1 = gp_Pnt(Distance, 0, 0)
        p2 = gp_Pnt(Distance + H1 / math.tan(alpha1 * math.pi / 180), 0, H1)
        p3 = gp_Pnt(Distance + L - H2 / math.tan(alpha2 * math.pi / 180), 0, H2)
        p4 = gp_Pnt(Distance + L, 0, 0)
        edge_p1_p2 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
        edge_p2_p3 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
        edge_p3_p4 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
        edge_p4_p1 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()
        wire_builder = BRepBuilderAPI_MakeWire()
        wire_builder.Add(edge_p1_p2)
        wire_builder.Add(edge_p2_p3)
        wire_builder.Add(edge_p3_p4)
        wire_builder.Add(edge_p4_p1)
        wire = wire_builder.Wire()

        # 检测wire是否是规范的封闭曲线

        pairs = get_non_adjacent_edge_pairs(wire)
        for e1, e2 in pairs:
            if edges_have_intersection(e1, e2):
                flag = False

        face = BRepBuilderAPI_MakeFace(wire).Face()
        # 进行2D倒圆角
        fillet2d = BRepFilletAPI_MakeFillet2d(face)

        # 遍历所有顶点，找到 `p2` 对应的 TopoDS_Vertex
        vertex_explorer = TopExp_Explorer(wire, TopAbs_VERTEX)
        p2_vertex = None
        while vertex_explorer.More():
            vertex = topods_Vertex(vertex_explorer.Current())  # 转换为 TopoDS_Vertex
            point = BRep_Tool.Pnt(vertex)  # 获取顶点坐标
            if abs(point.X() - p2.X()) < 1e-6 and abs(point.Y() - p2.Y()) < 1e-6:  # 判断是否是 p1
                p2_vertex = vertex
                break
            vertex_explorer.Next()
        fillet2d.AddFillet(p2_vertex, R1)

        # 遍历所有顶点，找到 `p3` 对应的 TopoDS_Vertex
        vertex_explorer = TopExp_Explorer(wire, TopAbs_VERTEX)
        p3_vertex = None
        while vertex_explorer.More():
            vertex = topods_Vertex(vertex_explorer.Current())  # 转换为 TopoDS_Vertex
            point = BRep_Tool.Pnt(vertex)  # 获取顶点坐标
            if abs(point.X() - p3.X()) < 1e-6 and abs(point.Y() - p3.Y()) < 1e-6:  # 判断是否是 p1
                p3_vertex = vertex
                break
            vertex_explorer.Next()
        fillet2d.AddFillet(p3_vertex, R2)
        filleted_face = fillet2d.Shape()

        solid1 = BRepPrimAPI_MakePrism(filleted_face, gp_Vec(0., W, 0)).Shape()
        solid2 = BRepPrimAPI_MakePrism(filleted_face, gp_Vec(0., -W, 0)).Shape()

        yi = BRepAlgoAPI_Fuse(solid1, solid2).Shape()


        rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
        for i in range(1, N):
            transform = gp_Trsf()
            transform.SetRotation(rotation_axis, math.radians(360 / N * i))
            rotated_shape = BRepBuilderAPI_Transform(yi, transform, True).Shape()
            yi = BRepAlgoAPI_Fuse(yi, rotated_shape).Shape()

        return flag, yi

    except Exception as e:
        flag = False
    finally:
        return flag, yi

def dish(Rout, L1, L2, alpha1, alpha2, r, R1, R2, L, scale):
    '''
    Parameters:
        Rout:外部半径
        L1:控制点1与前段的距离
        L2:控制点2与前段的距离
        alpha1:环向槽锥面倾角1
        alpha2:环向槽锥面倾角2
        r:环向槽圆弧半径
        R1:前段内孔
        R2:圆柱段内孔
        L:药柱长度
        scale:放缩因子
    '''
    # flag, dish = dish(40, 120, 60, 10, 8, 10, 20, 40, 500, 1)
    
    flag = True
    dish = TopoDS_Shape()
    L1 = L1 * scale
    L2 = L2 * scale
    alpha1 = alpha1 * scale
    alpha2 = alpha2 * scale
    r = r * scale
    R1 = R1 * scale
    R2 = R2 * scale
    p1 = gp_Pnt(L1, R1, 0)
    p2 = gp_Pnt(L2, R2, 0)

    try:

        line1 = gp_Lin2d(gp_Ax2d(gp_Pnt2d(L1, R1), gp_Dir2d(math.cos(math.radians(alpha1)), -math.sin(math.radians(alpha1)))))
        line2 = gp_Lin2d(gp_Ax2d(gp_Pnt2d(L2, R2), gp_Dir2d(math.cos(math.radians(alpha2)), -math.sin(math.radians(alpha2)))))

        circle_radius = r  # 圆的半径
        tolerance = 1e-6  # 计算容差

        # 包装直线为合格几何对象
        qualified_line1 = GccEnt_QualifiedLin(line1, GccEnt_Position(0))
        qualified_line2 = GccEnt_QualifiedLin(line2, GccEnt_Position(0))

        # 方法1: 使用GccAna_Circ2d2TanRad直接求解与两条直线相切的圆
        constructor = GccAna_Circ2d2TanRad(qualified_line1, qualified_line2,
                                           circle_radius, tolerance)
        solutions = []

        pcc1 = gp_Pnt(0, 0, 0)
        pcc2 = gp_Pnt(0, 0, 0)
        cc = None
        if constructor.IsDone():
            num_solutions = constructor.NbSolutions()
            # print(f"找到 {num_solutions} 个与两条直线相切的圆")
            min_y = 1e6
            for i in range(1, num_solutions + 1):
                circle = constructor.ThisSolution(i)
                center = circle.Location()
                if center.Y() < min_y:
                    min_y = center.Y()
                    pp1 = gp_Pnt(constructor.ThisSolution(i).Location().X(), constructor.ThisSolution(i).Location().Y(), 0)
                    circle = gp_Circ(gp_Ax2(pp1, gp_Dir(0, 0, -1)), r)

                    pc = gp_Pnt2d(0, 0)
                    constructor.Tangency1(i, pc)
                    pcc1 = gp_Pnt(pc.X(), pc.Y(), 0)
                    constructor.Tangency2(i, pc)
                    pcc2 = gp_Pnt(pc.X(), pc.Y(), 0)
                    arc_circle = GC_MakeArcOfCircle(circle, pcc1, pcc2, True)
                    cc = BRepBuilderAPI_MakeEdge(arc_circle.Value()).Edge()

            line1 = BRepBuilderAPI_MakeEdge(p1, pcc1).Edge()
            line2 = BRepBuilderAPI_MakeEdge(pcc2, p2).Edge()

            p00 = gp_Pnt(0, 0, 0)
            p0 = gp_Pnt(0, R1, 0)
            p3 = gp_Pnt(L, R2, 0)
            p4 = gp_Pnt(L, 0, 0)
            line00 = BRepBuilderAPI_MakeEdge(p00, p0).Edge()
            line0 = BRepBuilderAPI_MakeEdge(p0, p1).Edge()
            line3 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
            line4 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
            line5 = BRepBuilderAPI_MakeEdge(p4, p00).Edge()

            wire_builder = BRepBuilderAPI_MakeWire()
            wire_builder.Add(line00)
            wire_builder.Add(line0)
            wire_builder.Add(line1)
            wire_builder.Add(cc)
            wire_builder.Add(line2)
            wire_builder.Add(line3)
            wire_builder.Add(line4)
            wire_builder.Add(line5)
            wire = wire_builder.Wire()

            # 检测wire是否是规范的封闭曲线
            flag = True
            pairs = get_non_adjacent_edge_pairs(wire)
            for e1, e2 in pairs:
                if edges_have_intersection(e1, e2):
                    flag = False

            if pcc2.Y() >= Rout:
                flag = False
                return flag, dish

            face = BRepBuilderAPI_MakeFace(wire).Face()
            axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0))  # 绕 X 轴旋转 (gp_Ax1 类型)
            solid1 = BRepPrimAPI_MakeRevol(face, axis, math.radians(180)).Shape()  # 旋转90度 (7种.5708 rad)
            solid2 = BRepPrimAPI_MakeRevol(face, axis, math.radians(-180)).Shape()
            dish = BRepAlgoAPI_Fuse(solid1, solid2).Shape()
            return flag, dish

    except Exception as e:
        flag = False
    finally:
        return flag, dish

        # display, start_display, add_menu, add_function_to_menu = init_display()
        # display.DisplayShape(solid, update=True)
        # # display.DisplayShape(p2, update=True)
        # # display.DisplayShape(line00, update=True)
        # # display.DisplayShape(line0, update=True)
        # # display.DisplayShape(line1, update=True)
        # # display.DisplayShape(line2, update=True)
        # # display.DisplayShape(line3, update=True)
        # # display.DisplayShape(line4, update=True)
        # # display.DisplayShape(line5, update=True)
        # # display.DisplayShape(cc, update=True)
        # # display.DisplayShape(pcc1, update=True)
        # # display.DisplayShape(pcc2, update=True)
        # view = display.View
        # view.SetProj(0, 0, 1)
        # start_display()

def wheelPort(Rin, e, Rout, L, N):
    """
    Args:
        Rin: 药柱内径
        e: 肉厚
        Rout: 药柱外径
        L: 药柱长度
        N: 对称数
    """
    angle =  math.pi / N
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(0, (Rin+2*e)*math.sin(math.radians(180 / N)), (Rin+2*e)*math.cos(math.radians(180 / N)))
    p3 = gp_Pnt(0, 0, Rin+2*e)
    p4 = gp_Pnt(0, (Rout-e)*math.sin(math.radians(180 / N)), (Rout-e)*math.cos(math.radians(180 / N)))
    p5 = gp_Pnt(0, 0, Rout-e)
    p6 = gp_Pnt(0, 0, 0)
    p7 = gp_Pnt(0, 0, 0)

    circle_axis = gp_Ax2(p1, gp_Dir(1, 0, 0))
    circle = gp_Circ(circle_axis, Rin+2*e)
    c = GC_MakeArcOfCircle(circle, p2, p3, True)
    rin_edge = BRepBuilderAPI_MakeEdge(c.Value()).Edge()

    circle_axis = gp_Ax2(p1, gp_Dir(1, 0, 0))
    circle = gp_Circ(circle_axis, Rout-e)
    c = GC_MakeArcOfCircle(circle, p4, p5, True)
    rout_edge = BRepBuilderAPI_MakeEdge(c.Value()).Edge()


    edge = BRepBuilderAPI_MakeEdge(p1, p4).Edge()
    distance_e = e
    direction_vector = gp_Vec(0, -math.cos(math.radians(180 / N)), math.sin(math.radians(180 / N)))
    translation_vector = direction_vector.Normalized().Scaled(distance_e)
    translation_trsf = gp_Trsf()
    translation_trsf.SetTranslation(translation_vector)
    trsf_api = BRepBuilderAPI_Transform(edge, translation_trsf)
    translated_edge = trsf_api.Shape()


    plane = gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0))
    curve3d_1, u_min1, u_max1 = BRep_Tool.Curve(TopoDS.topods_Edge(translated_edge))
    curve3d_2, u_min2, u_max2 = BRep_Tool.Curve(rin_edge)
    curve3d_3, u_min3, u_max3 = BRep_Tool.Curve(rout_edge)
    curve2d_1 = GeomAPI.geomapi_To2d(curve3d_1, plane)
    curve2d_2 = GeomAPI.geomapi_To2d(curve3d_2, plane)
    curve2d_3 = GeomAPI.geomapi_To2d(curve3d_3, plane)
    intersector = Geom2dAPI_InterCurveCurve(curve2d_1, curve2d_2)
    num_intersections = intersector.NbPoints()
    for i in range(1, num_intersections + 1):
        point2d = intersector.Point(i)
        if point2d.X() < 0 and point2d.Y() < 0:
            p6 = gp_Pnt(0, -point2d.Y(), -point2d.X())

    intersector = Geom2dAPI_InterCurveCurve(curve2d_1, curve2d_3)
    num_intersections = intersector.NbPoints()
    for i in range(1, num_intersections + 1):
        point2d = intersector.Point(i)
        if point2d.X() < 0 and point2d.Y() < 0:
            p7 = gp_Pnt(0, -point2d.Y(), -point2d.X())


    circle_axis = gp_Ax2(p1, gp_Dir(1, 0, 0))
    circle = gp_Circ(circle_axis, Rin+2*e)
    c = GC_MakeArcOfCircle(circle, p6, p3, True)
    edge_1 = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    edge_2 = BRepBuilderAPI_MakeEdge(p6, p7).Edge()

    circle_axis = gp_Ax2(p1, gp_Dir(1, 0, 0))
    circle = gp_Circ(circle_axis, Rout-e)
    c = GC_MakeArcOfCircle(circle, p7, p5, True)
    edge_3 = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    edge_4 = BRepBuilderAPI_MakeEdge(p5, p3).Edge()

    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(edge_1)
    wire_builder.Add(edge_2)
    wire_builder.Add(edge_3)
    wire_builder.Add(edge_4)
    wire = wire_builder.Wire()

    # 检测wire是否是规范的封闭曲线
    flag = True
    pairs = get_non_adjacent_edge_pairs(wire)
    for e1, e2 in pairs:
        if edges_have_intersection(e1, e2):
            flag = False

    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid = BRepPrimAPI_MakePrism(face, gp_Vec(L, 0, 0)).Shape()

    mirror_plane = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
    transform = gp_Trsf()
    transform.SetMirror(mirror_plane)
    mirror_builder = BRepBuilderAPI_Transform(solid, transform, True)
    mirrored_solid = mirror_builder.Shape()
    fused_shape = BRepAlgoAPI_Fuse(solid, mirrored_solid).Shape()


    # p1 = gp_Pnt(0, 0, 0)
    # p2 = gp_Pnt(0, Rin*math.sin(math.radians(180 / N)), Rin*math.cos(math.radians(180 / N)))
    # p3 = gp_Pnt(0, -Rin*math.sin(math.radians(180 / N)),Rin*math.cos(math.radians(180 / N)))
    # circle = gp_Circ(circle_axis, Rin)
    # c = GC_MakeArcOfCircle(circle, p2, p3, True)
    # edge_1 = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    # edge_2 = BRepBuilderAPI_MakeEdge(p2, p1).Edge()
    # edge_3 = BRepBuilderAPI_MakeEdge(p3, p1).Edge()
    # wire_builder = BRepBuilderAPI_MakeWire()
    # wire_builder.Add(edge_1)
    # wire_builder.Add(edge_2)
    # wire_builder.Add(edge_3)
    # wire = wire_builder.Wire()
    # innerface = BRepBuilderAPI_MakeFace(wire).Face()
    # innersolid = BRepPrimAPI_MakePrism(innerface, gp_Vec(L, 0, 0)).Shape()
    #
    # fused_shape = BRepAlgoAPI_Fuse(fused_shape, innersolid).Shape()
    # display, start_display, add_menu, add_function_to_menu = init_display()
    # display.DisplayShape(innersolid, update=True)
    # display.DisplayShape(fused_shape, update=True)
    # display.DisplayShape(edge_3, update=True)
    # display.DisplayShape(edge_4, update=True)
    # display.DisplayShape(p6, update=True)
    # display.DisplayShape(p7, update=True)
    # view = display.View
    # view.SetProj(1, 0, 0)
    # start_display()
    rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    for i in range(1, N):
        transform = gp_Trsf()
        transform.SetRotation(rotation_axis, math.radians(360 / N * i))
        rotated_shape = BRepBuilderAPI_Transform(fused_shape, transform, True).Shape()
        fused_shape = BRepAlgoAPI_Fuse(fused_shape, rotated_shape).Shape()

    return flag, fused_shape

def umbrella(Ls, H, r):
    """

    Args:
        Ls: 与前端距离
        H: 高度
        r: 过渡半径
    """
    p1 = gp_Pnt(Ls, 0, 0)
    p2 = gp_Pnt(Ls, H-r, 0)
    p3 = gp_Pnt(Ls+r, H, 0)
    p4 = gp_Pnt(Ls+2*r, H-r, 0)
    p5 = gp_Pnt(Ls+2*r, 0, 0)
    line1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    arc_circle = GC_MakeArcOfCircle(p2, p3, p4)
    cc = BRepBuilderAPI_MakeEdge(arc_circle.Value()).Edge()
    line2 = BRepBuilderAPI_MakeEdge(p4, p5).Edge()
    line3 = BRepBuilderAPI_MakeEdge(p5, p1).Edge()

    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(line1)
    wire_builder.Add(cc)
    wire_builder.Add(line2)
    wire_builder.Add(line3)
    wire = wire_builder.Wire()

    # 检测wire是否是规范的封闭曲线
    flag = True
    pairs = get_non_adjacent_edge_pairs(wire)
    for e1, e2 in pairs:
        if edges_have_intersection(e1, e2):
            flag = False

    face = BRepBuilderAPI_MakeFace(wire).Face()
    axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(-1, 0, 0))  # 绕 X 轴旋转 (gp_Ax1 类型)
    solid1 = BRepPrimAPI_MakeRevol(face, axis, math.radians(180)).Shape()  # 旋转90度 (7种.5708 rad)
    solid2 = BRepPrimAPI_MakeRevol(face, axis, math.radians(-180)).Shape()
    solid = BRepAlgoAPI_Fuse(solid1, solid2).Shape()

    return flag, solid

def slottedTube(Rout, Rin, L, N, width, length):
    """

    Args:
        Rout: 药柱外径
        Rin: 药柱内径
        L: 药柱长度
        N: 对称数
        width: 开槽宽度的一半
        length: 开槽的长度
    """
    inner_ = inner(Rin, L, 1, 1)

    p1 = gp_Pnt(L, 0, 0)
    p2 = gp_Pnt(L, 0, Rout)
    p3 = gp_Pnt(L-length, 0, Rout)
    p4 = gp_Pnt(L-length, 0, 0)

    line1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    line2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    line3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
    line4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()

    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(line1)
    wire_builder.Add(line2)
    wire_builder.Add(line3)
    wire_builder.Add(line4)
    wire = wire_builder.Wire()

    # 检测wire是否是规范的封闭曲线
    flag = True
    pairs = get_non_adjacent_edge_pairs(wire)
    for e1, e2 in pairs:
        if edges_have_intersection(e1, e2):
            flag = False
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid1 = BRepPrimAPI_MakePrism(face, gp_Vec(0, width, 0)).Shape()
    solid2 = BRepPrimAPI_MakePrism(face, gp_Vec(0, -width, 0)).Shape()
    slotted = BRepAlgoAPI_Fuse(solid1, solid2).Shape()
    rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    for i in range(1, N):
        transform = gp_Trsf()
        transform.SetRotation(rotation_axis, math.radians(360 / N * i))
        rotated_shape = BRepBuilderAPI_Transform(slotted, transform, True).Shape()
        slotted = BRepAlgoAPI_Fuse(slotted, rotated_shape).Shape()
    slotted = BRepAlgoAPI_Fuse(slotted, inner_).Shape()

    return flag, slotted

def slottedTube1(Rout, Rin, L, N, width, length, Rout1):
    """

    Args:
        Rout: 药柱外径
        Rin: 药柱内径
        L: 药柱长度
        N: 对称数
        width: 开槽宽度的一半
        length: 开槽的长度
    """
    inner_ = inner(Rin, L, 1, 1)

    p1 = gp_Pnt(L, 0, 0)
    p2 = gp_Pnt(L, 0, Rout1)
    p3 = gp_Pnt(L-length, 0, Rout)
    p4 = gp_Pnt(L-length, 0, 0)

    line1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    line2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    line3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
    line4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()

    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(line1)
    wire_builder.Add(line2)
    wire_builder.Add(line3)
    wire_builder.Add(line4)
    wire = wire_builder.Wire()

    # 检测wire是否是规范的封闭曲线
    flag = True
    pairs = get_non_adjacent_edge_pairs(wire)
    for e1, e2 in pairs:
        if edges_have_intersection(e1, e2):
            flag = False
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid1 = BRepPrimAPI_MakePrism(face, gp_Vec(0, width, 0)).Shape()
    solid2 = BRepPrimAPI_MakePrism(face, gp_Vec(0, -width, 0)).Shape()
    slotted = BRepAlgoAPI_Fuse(solid1, solid2).Shape()
    rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    for i in range(1, N):
        transform = gp_Trsf()
        transform.SetRotation(rotation_axis, math.radians(360 / N * i))
        rotated_shape = BRepBuilderAPI_Transform(slotted, transform, True).Shape()
        slotted = BRepAlgoAPI_Fuse(slotted, rotated_shape).Shape()
    slotted = BRepAlgoAPI_Fuse(slotted, inner_).Shape()

    return flag, slotted

def star(l, r, epsilon, N, theta, L, rc):
    """
    Make a star of grain.
    Parameters:
        l：特征尺寸
        r: 星槽圆弧半径
        epsilon: 星角系数
        N: 星角数
        theta: 星边夹角
        L: 药柱长度
        rc: 星根圆弧半径
    """
    # 画出外层圆形，找到了外层圆弧xingding
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(0, 0, l + r)
    angle_one = (1 - epsilon) * math.pi / N
    circle_axis = gp_Ax2(p1, gp_Dir(1, 0, 0))
    circle = gp_Circ(circle_axis, l + r)
    c = GC_MakeArcOfCircle(circle, 0, angle_one, True)
    arc_edge = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    xingding = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    LastVertex = TopExp.topexp_LastVertex(arc_edge)
    p3 = BRep_Tool.Pnt(LastVertex)

    # 找到星槽过度圆弧xingcao
    circle = gp_Circ(circle_axis, l)
    c = GC_MakeArcOfCircle(circle, 0, angle_one, True)
    arc_edge = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    LastVertex = TopExp.topexp_LastVertex(arc_edge)
    p4 = BRep_Tool.Pnt(LastVertex)
    c = gp_Circ2d(gp_Ax2d(gp_Pnt2d(p4.XYZ().Y(), p4.XYZ().Z()), gp_Dir2d(1, 0)), p3.Distance(p4))
    theLine = gp_Lin2d(
        gp_Ax2d(gp_Pnt2d(0, 0), gp_Dir2d(-math.sin(math.radians(180 / N)), math.cos(math.radians(180 / N)))))
    solver = GccAna_Lin2dTanObl(GccEnt_QualifiedCirc(c, GccEnt_Position(0)), theLine, (180 - theta) / 180 * math.pi)
    p5 = gp_Pnt(0, solver.ThisSolution(1).Location().X(), solver.ThisSolution(1).Location().Y())
    C = gp_Circ(gp_Ax2(gp_Pnt(0, p4.XYZ().Y(), p4.XYZ().Z()), gp_Dir(1, 0, 0)), p3.Distance(p4))
    circle = GC_MakeArcOfCircle(C, p3, p5, False).Value()
    xingcao = BRepBuilderAPI_MakeEdge(circle).Edge()

    # 另外两条边构成封闭曲线
    line1 = gp_Lin(p5, gp_Dir(0, solver.ThisSolution(1).Direction().X(), solver.ThisSolution(1).Direction().Y()))
    geom_line1 = Geom_Line(line1)
    ais_line1 = AIS_Line(geom_line1)
    line2 = gp_Lin(gp_Pnt(0, 0, 0), gp_Dir(0, -math.sin(math.radians(180 / N)), math.cos(math.radians(180 / N))))
    geom_line2 = Geom_Line(line2)
    ais_line2 = AIS_Line(geom_line2)

    # 找到星根圆弧xinggen
    p6 = gp_Pnt(0, 0, 0)
    p66 = gp_Pnt(0, 0, 0)
    GeomAPI_ExtremaCurveCurve(geom_line1, geom_line2).NearestPoints(p6, p66)
    # c = gp_Circ2d(gp_Ax2d(gp_Pnt2d(p4.XYZ().Y(), p4.XYZ().Z()), gp_Dir2d(1, 0)), 3)
    line = GccEnt_QualifiedLin(gp_Lin2d(gp_Pnt2d(p5.Y(), p5.Z()), gp_Dir2d(p6.Y() - p5.Y(), p6.Z() - p5.Z())),
                               GccEnt_Position(0))
    onLine = gp_Lin2d(gp_Pnt2d(p6.Y(), p6.Z()),
                      gp_Dir2d(-math.sin(math.radians(180 / N)), math.cos(math.radians(180 / N))))
    solver = GccAna_Circ2dTanOnRad(line, onLine, rc, 1e-7)
    pc = gp_Pnt2d(0, 0)
    solver.Tangency1(1, pc)
    # 切点pcc
    pcc = gp_Pnt(0, pc.X(), pc.Y())
    pp1 = gp_Pnt(0, solver.ThisSolution(1).Location().X(), solver.ThisSolution(1).Location().Y())
    c = gp_Circ(gp_Ax2(pp1, gp_Dir(1, 0, 0)), rc)
    cc = BRepBuilderAPI_MakeEdge(c).Edge()
    line = gp_Lin(gp_Pnt(0, 0, 0),
                  gp_Dir(0, solver.ThisSolution(1).Location().X(), solver.ThisSolution(1).Location().Y()))
    extrema = GeomAPI_ExtremaCurveCurve(Geom_Circle(c), Geom_Line(line))
    pppp = []
    if extrema.NbExtrema() > 0:
        for i in range(1, extrema.NbExtrema() + 1):
            p1_ = gp_Pnt(0, 0, 0)
            p2_ = gp_Pnt(0, 0, 0)
            extrema.Points(i, p1_, p2_)
            dist = p1_.Distance(p2_)
            if dist < 1e-6:
                pppp.append(p1_)
    dist1 = pppp[0].Distance(p6)
    dist2 = pppp[1].Distance(p6)
    if dist2 < dist1:
        p7 = pppp[1]
    else:
        p7 = pppp[0]
    c = GC_MakeArcOfCircle(c, p7, pcc, True)
    xinggen = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    edge_6_1 = BRepBuilderAPI_MakeEdge(p6, p1).Edge()
    edge_1_2 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    edge_5_c = BRepBuilderAPI_MakeEdge(p5, pcc).Edge()
    edge_7_1 = BRepBuilderAPI_MakeEdge(p7, p1).Edge()

    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(edge_1_2)
    wire_builder.Add(xingding)
    wire_builder.Add(topods_Edge(xingcao))
    wire_builder.Add(edge_5_c)
    wire_builder.Add(xinggen)
    wire_builder.Add(edge_7_1)
    wire = wire_builder.Wire()

    # 检测wire是否是规范的封闭曲线
    flag = True
    pairs = get_non_adjacent_edge_pairs(wire)
    for e1, e2 in pairs:
        if edges_have_intersection(e1, e2):
            flag = False

    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid = BRepPrimAPI_MakePrism(face, gp_Vec(L, 0, 0)).Shape()

    mirror_plane = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
    transform = gp_Trsf()
    transform.SetMirror(mirror_plane)
    mirror_builder = BRepBuilderAPI_Transform(solid, transform, True)
    mirrored_solid = mirror_builder.Shape()
    fused_shape = BRepAlgoAPI_Fuse(solid, mirrored_solid).Shape()

    rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    for i in range(1, N):
        transform = gp_Trsf()
        transform.SetRotation(rotation_axis, math.radians(360 / N * i))
        rotated_shape = BRepBuilderAPI_Transform(fused_shape, transform, True).Shape()
        fused_shape = BRepAlgoAPI_Fuse(fused_shape, rotated_shape).Shape()
    return flag, fused_shape

def wheel(R, e1, N, theta, epsilon, r, r1, r2, h, L):
    """
    Make a wheel of grain.
    Parameters:
        # R: 外部半径
        # e1: 肉厚
        # N: 对称数
        # theta: 轮臂角
        # epsilon: 角度系数
        # r: 过渡圆弧半径
        # r1: 轮臂角圆弧半径
        # r2: 轮臂圆弧半径
        # h: 轮臂高度
        # L: 长度
    """
    # 画出外层圆形，找到了外层圆弧cheding
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(0, 0, R - e1)
    angle_one = math.pi / N
    circle_axis = gp_Ax2(p1, gp_Dir(1, 0, 0))
    circle = gp_Circ(circle_axis, R - e1)
    c = GC_MakeArcOfCircle(circle, 0, angle_one, True)

    angle_two = epsilon * math.pi / N
    circle_axis = gp_Ax2(p1, gp_Dir(1, 0, 0))
    circle = gp_Circ(circle_axis, R - e1)
    c = GC_MakeArcOfCircle(circle, angle_two, angle_one, True)
    cheding = BRepBuilderAPI_MakeEdge(c.Value()).Edge()
    FirstVertex = TopExp.topexp_FirstVertex(cheding)
    LastVertex = TopExp.topexp_LastVertex(cheding)
    p3 = BRep_Tool.Pnt(LastVertex)
    p4 = BRep_Tool.Pnt(FirstVertex)

    line1 = gp_Lin2d(gp_Ax2d(gp_Pnt2d(0, 0), gp_Dir2d(p4.Y(), p4.Z())))
    circle1 = gp_Circ2d(gp_Ax2d(gp_Pnt2d(0, 0), gp_Dir2d(1, 0)), R - e1)
    c1 = GccAna_Circ2dTanOnRad(GccEnt_QualifiedCirc(circle1, GccEnt_Position(0)), line1, r, 1e-7)
    p5 = gp_Pnt(0, c1.ThisSolution(1).Location().X(), c1.ThisSolution(1).Location().Y())

    circle2 = gp_Circ2d(
        gp_Ax2d(gp_Pnt2d(c1.ThisSolution(1).Location().X(), c1.ThisSolution(1).Location().Y()), gp_Dir2d(1, 0)), r)
    line2 = gp_Lin2d(gp_Pnt2d(0, 0), gp_Dir2d(0, -1))
    c = GccAna_Lin2dTanPar(GccEnt_QualifiedCirc(circle2, GccEnt_Position(0)), line2)
    if c.ThisSolution(1).Location().X() < c.ThisSolution(2).Location().X():
        p6 = gp_Pnt(0, c.ThisSolution(2).Location().X(), c.ThisSolution(2).Location().Y())
    else:
        p6 = gp_Pnt(0, c.ThisSolution(1).Location().X(), c.ThisSolution(1).Location().Y())
    d = gp_Circ(
        gp_Ax2(gp_Pnt(0, c1.ThisSolution(1).Location().X(), c1.ThisSolution(1).Location().Y()), gp_Dir(1, 0, 0)), r)
    d = GC_MakeArcOfCircle(d, p6, p4, True).Value()
    dd = BRepBuilderAPI_MakeEdge(d).Edge()

    start_point = gp_Pnt(1, 1, 1)
    direction = gp_Dir(1, 2, 3)
    vec = gp_Vec(gp_Dir(0, 0, -1))
    vec.Scale(h)
    p7 = gp_Pnt(p6.XYZ() + vec.XYZ())
    segment = GC_MakeSegment(p6, p7).Value()
    segment = BRepBuilderAPI_MakeEdge(segment).Edge()

    p8 = gp_Pnt(0, p7.Y() + r2, p7.Z())
    circle3 = gp_Circ2d(gp_Ax2d(gp_Pnt2d(p8.Y(), p8.Z()), gp_Dir2d(1, 0)), r2)
    cc = gp_Circ(gp_Ax2(gp_Pnt(0, p8.Y(), p8.Z()), gp_Dir(1, 0, 0)), r2)
    line3 = gp_Lin2d(gp_Pnt2d(0, 0), gp_Dir2d(-math.sin(math.radians(theta)), math.cos(math.radians(theta))))
    c3 = GccAna_Lin2dTanPar(GccEnt_QualifiedCirc(circle3, GccEnt_Position(0)), line3)
    if c3.ThisSolution(1).Location().X() < c3.ThisSolution(2).Location().X():
        p9 = gp_Pnt(0, c3.ThisSolution(1).Location().X(), c3.ThisSolution(1).Location().Y())
    else:
        p9 = gp_Pnt(0, c3.ThisSolution(2).Location().X(), c3.ThisSolution(2).Location().Y())

    d = GC_MakeArcOfCircle(gp_Circ(gp_Ax2(gp_Pnt(0, p8.Y(), p8.Z()), gp_Dir(1, 0, 0)), r2), p7, p9, True).Value()
    ddd = BRepBuilderAPI_MakeEdge(d).Edge()

    geom_line1 = Geom_Line(gp_Lin(p9, gp_Dir(0, -math.sin(math.radians(theta)), math.cos(math.radians(theta)))))
    geom_line2 = Geom_Line(gp_Lin(p1, gp_Dir(0, 0, 1)))
    # 使用 GeomAPI_IntCS 查找交点（Curve-Surface）
    extrema = GeomAPI_ExtremaCurveCurve(geom_line1, geom_line2)
    p10 = gp_Pnt(0, 0, 0)
    p100 = gp_Pnt(0, 0, 0)
    if extrema.NbExtrema() > 0:
        for i in range(1, extrema.NbExtrema() + 1):
            extrema.Points(i, p10, p100)

    line4 = gp_Lin2d(gp_Pnt2d(p9.Y(), p9.Z()), gp_Dir2d(-math.sin(math.radians(theta)), math.cos(math.radians(theta))))
    line5 = gp_Lin2d(gp_Pnt2d(0, 0), gp_Dir2d(0, 1))
    c = GccAna_Circ2dTanOnRad(GccEnt_QualifiedLin(line4, GccEnt_Position(0)), line5, r1, 1e-7)
    pc = gp_Pnt2d(0, 0)
    if c.ThisSolution(1).Location().Y() > c.ThisSolution(2).Location().Y():
        p11 = gp_Pnt(0, c.ThisSolution(1).Location().X(), c.ThisSolution(1).Location().Y())
        c.Tangency1(1, pc)
        pcc = gp_Pnt(0, pc.X(), pc.Y())
    else:
        p11 = gp_Pnt(0, c.ThisSolution(2).Location().X(), c.ThisSolution(2).Location().Y())
        c.Tangency1(2, pc)
        pcc = gp_Pnt(0, pc.X(), pc.Y())

    geom_line1 = Geom_Circle(gp_Circ(gp_Ax2(p11, gp_Dir(1, 0, 0)), r1))
    geom_line2 = Geom_Line(gp_Lin(p1, gp_Dir(0, 0, 1)))
    # 使用 GeomAPI_IntCS 查找交点（Curve-Surface）
    extrema = GeomAPI_ExtremaCurveCurve(geom_line1, geom_line2)
    p12 = gp_Pnt(0, 0, 0)
    p120 = gp_Pnt(0, 0, 0)
    if extrema.NbExtrema() > 0:
        for i in range(1, extrema.NbExtrema() + 1):
            extrema.Points(i, p12, p120)

    c = gp_Circ(gp_Ax2(p11, gp_Dir(1, 0, 0)), r1)
    c = GC_MakeArcOfCircle(c, pcc, p12, True).Value()
    dddd = BRepBuilderAPI_MakeEdge(c).Edge()
    edge9_cc = BRepBuilderAPI_MakeEdge(p9, pcc).Edge()
    edge12_1 = BRepBuilderAPI_MakeEdge(p12, p1).Edge()
    edge1_3 = BRepBuilderAPI_MakeEdge(p1, p3).Edge()

    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(cheding)
    wire_builder.Add(dd)
    wire_builder.Add(segment)
    wire_builder.Add(ddd)
    wire_builder.Add(edge9_cc)
    wire_builder.Add(dddd)
    wire_builder.Add(edge12_1)
    wire_builder.Add(edge1_3)
    wire = wire_builder.Wire()
    # 检测wire是否是规范的封闭曲线
    flag = True
    pairs = get_non_adjacent_edge_pairs(wire)
    for e1, e2 in pairs:
        if edges_have_intersection(e1, e2):
            flag = False
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid = BRepPrimAPI_MakePrism(face, gp_Vec(L, 0, 0)).Shape()
    mirror_plane = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
    transform = gp_Trsf()
    transform.SetMirror(mirror_plane)
    mirror_builder = BRepBuilderAPI_Transform(solid, transform, True)
    mirrored_solid = mirror_builder.Shape()
    fused_shape = BRepAlgoAPI_Fuse(solid, mirrored_solid).Shape()
    rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    for i in range(1, N):
        transform = gp_Trsf()
        transform.SetRotation(rotation_axis, math.radians(360 / N * i))
        rotated_shape = BRepBuilderAPI_Transform(fused_shape, transform, True).Shape()
        fused_shape = BRepAlgoAPI_Fuse(fused_shape, rotated_shape).Shape()

    fused_shape = rotate_solid_around_x(fused_shape, angle_degrees=180 / N, origin=(0, 0, 0))

    return flag, fused_shape

def multiHoles(L, l, r, N):
    """
    Args:
        L: 药柱长度
        l: 距离
        r: 半径
    """
    fused_shape = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, l), gp_Dir(1, 0, 0)), r, L).Shape()

    rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    for i in range(1, N):
        transform = gp_Trsf()
        transform.SetRotation(rotation_axis, math.radians(360 / N * i))
        rotated_shape = BRepBuilderAPI_Transform(fused_shape, transform, True).Shape()
        fused_shape = BRepAlgoAPI_Fuse(fused_shape, rotated_shape).Shape()

    fused_shape = rotate_solid_around_x(fused_shape, angle_degrees=360 / N, origin=(0, 0, 0))
    return True, fused_shape

def anchor(L, N, l, r1, epsilon, d):
    beta = 360/N

    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(0, 0, l+r1)
    p3 = gp_Pnt(0, (l+r1)*math.sin(math.radians(1-epsilon)*beta), (l+r1)*math.cos(math.radians(1-epsilon)*beta))
    p4 = gp_Pnt(0, l*math.sin(math.radians(1-epsilon)*beta), l*math.cos(math.radians(1-epsilon)*beta))
    p5 = gp_Pnt(0, (l-r1)*math.sin(math.radians(1 - epsilon) * beta), (l-r1)*math.cos(math.radians(1 - epsilon) * beta))
    p6 = gp_Pnt(0, d, d/math.tan(math.radians(epsilon*beta)))

    line1 = gp_Lin(p5, gp_Dir(0, math.cos(math.radians(1-epsilon)*beta), -math.sin(math.radians(1-epsilon)*beta)))
    line2 = gp_Lin(p6, gp_Dir(0, 0, 1))
    geom_line1 = Geom_Line(line1)
    geom_line2 = Geom_Line(line2)
    extrema = GeomAPI_ExtremaCurveCurve(geom_line1, geom_line2)
    p01 = gp_Pnt(0, 0, 0)
    p02 = gp_Pnt(0, 0, 0)
    if extrema.NbExtrema() > 0:
        for i in range(1, extrema.NbExtrema() + 1):
            extrema.Points(i, p01, p02)

    edge_1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()

    circle_axis = gp_Ax2(p1, gp_Dir(-1, 0, 0))
    circle = gp_Circ(circle_axis, r1+l)
    c = GC_MakeArcOfCircle(circle, p2, p3, True).Value()
    edge_2 = BRepBuilderAPI_MakeEdge(c).Edge()

    circle_axis = gp_Ax2(p4, gp_Dir(-1, 0, 0))
    circle = gp_Circ(circle_axis, r1)
    c = GC_MakeArcOfCircle(circle, p3, p5, True).Value()
    edge_3 = BRepBuilderAPI_MakeEdge(c).Edge()

    edge_4 = BRepBuilderAPI_MakeEdge(p5, p01).Edge()
    edge_5 = BRepBuilderAPI_MakeEdge(p01, p6).Edge()
    edge_6 = BRepBuilderAPI_MakeEdge(p6, p1).Edge()

    wire_builder = BRepBuilderAPI_MakeWire()
    wire_builder.Add(edge_1)
    wire_builder.Add(edge_2)
    wire_builder.Add(edge_3)
    wire_builder.Add(edge_4)
    wire_builder.Add(edge_5)
    wire_builder.Add(edge_6)
    wire = wire_builder.Wire()
    # 检测wire是否是规范的封闭曲线
    flag = True
    pairs = get_non_adjacent_edge_pairs(wire)
    for e1, e2 in pairs:
        if edges_have_intersection(e1, e2):
            flag = False
    face = BRepBuilderAPI_MakeFace(wire).Face()
    solid = BRepPrimAPI_MakePrism(face, gp_Vec(L, 0, 0)).Shape()
    mirror_plane = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
    transform = gp_Trsf()
    transform.SetMirror(mirror_plane)
    mirror_builder = BRepBuilderAPI_Transform(solid, transform, True)
    mirrored_solid = mirror_builder.Shape()
    fused_shape = BRepAlgoAPI_Fuse(solid, mirrored_solid).Shape()
    rotation_axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
    for i in range(1, N):
        transform = gp_Trsf()
        transform.SetRotation(rotation_axis, math.radians(360 / N * i))
        rotated_shape = BRepBuilderAPI_Transform(fused_shape, transform, True).Shape()
        fused_shape = BRepAlgoAPI_Fuse(fused_shape, rotated_shape).Shape()

    # display, start_display, add_menu, add_function_to_menu = init_display()
    # display.DisplayShape(edge_1, update=True)
    # display.DisplayShape(edge_2, update=True)
    # display.DisplayShape(edge_3, update=True)
    # display.DisplayShape(edge_4, update=True)
    # display.DisplayShape(edge_5, update=True)
    # display.DisplayShape(edge_6, update=True)
    # # display.DisplayShape(fused_shape, update=True)
    # # display.DisplayShape(p1, update=True)
    # # display.DisplayShape(p2, update=True)
    # # display.DisplayShape(p3, update=True)
    # # display.DisplayShape(p4, update=True)
    # # display.DisplayShape(p01, update=True)
    # # display.DisplayShape(p02, update=True)
    # view = display.View
    # view.SetProj(1, 0, 0)
    # start_display()

    return flag, fused_shape


def generateGrainModel(feature: list, outer, path, showXinmo=False, showGrain=False):
    """
    对药柱作布尔运算，并导出grain.step, grain.obj。注意：这里的grain.step包含了心模
    Parameters:
        feature - 心模特征（一个列表）
        outer - 外轮廓
        path - 保存路径（含名称）
    Return:
        grain - 药柱
        grainVolume - 药柱体积
        gasVolume - 空腔体积
        Flag - 是否正常生成
    """
    try:
        Flag =True
        grainVolume, gasVolume = 0, 0
        xinmo = feature[0]
        if len(feature) != 1:
            for i in range(1, len(feature)):
                xinmo = BRepAlgoAPI_Fuse(xinmo, feature[i]).Shape()

        grain = BRepAlgoAPI_Cut(outer, xinmo).Shape()

        # 计算初始自由容积
        grainVolume = properties.compute_solid_volume(grain)
        gasVolume = properties.compute_solid_volume(outer) - grainVolume
        # 导出药柱的step格式模型
        exportSTEP([grain], path + "/" + "grain.step")
        # export_to_stl(grain, path + "/" + "grain.stl")
        # export_obj(grain, path + "/" + "grain.obj")

        # 导出实际心膜的step格式模型
        actual_xinmo = BRepAlgoAPI_Common(outer, xinmo).Shape()

        if showXinmo:
            display(actual_xinmo)

        if showGrain:
            display(grain)

        exportSTEP([actual_xinmo], savePath= path + "/" + "xinmo.step")
        export_to_stl(actual_xinmo, path + "/" + "xinmo.stl")
        export_obj(actual_xinmo, path + "/" + "xinmo.obj")
        export_obj(grain, path + "/" + "grain.obj")

        return grain, grainVolume, gasVolume, Flag
    except Exception as e:
        Flag = False
        return TopoDS_Shape(), grainVolume, gasVolume, Flag

def mesh_step_2d(step_path: str, msh_path: str, mesh_size: float = 5.0) -> None:
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 1)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", mesh_size)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", mesh_size)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)

    try:
        gmsh.open(step_path)

        # STEP import creates CAD entities; synchronize before meshing.
        gmsh.model.occ.synchronize()

        gmsh.model.mesh.generate(2)
        gmsh.write(msh_path)
    finally:
        gmsh.finalize()

def generateGrainMesh(geometryPath = "grainWithXinmo.step", meshSavePath = "grain.msh", showOCC=False, showMesh=False):
    """
    生成网格
    :param geometryPath:药柱step文件全路径，需要同时包含药柱外轮廓与芯模，药柱外轮廓位于最后
    :param meshSavePath: 网格保存全路径
    :param showOCC: 是否展示导入的step，默认不展示
    :param showMesh: 是否展示生成的网格，默认不展示
    :return: None
    """
    ## 做一些几何拓扑处理
    gmsh.initialize()
    gmsh.option.setNumber("General.Verbosity", 0)
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.open(geometryPath)
    gmsh.option.setNumber("Geometry.Surfaces", 1)
    gmsh.option.setNumber("Geometry.SurfaceType", 2)

    ## 上色，药柱颜色全部为（255, 128, 0），芯模颜色全部为（255, 12, 28）
    for volume in gmsh.model.occ.getEntities(3)[-1:]:
        surfaceLoopTags, surfaceTags = gmsh.model.occ.getSurfaceLoops(volume[1])
        for surfaceTag in surfaceTags[0]:
            gmsh.model.setColor([(2, surfaceTag)], 255, 128, 0)

    for volume in gmsh.model.occ.getEntities(3)[:-1]:
        surfaceLoopTags, surfaceTags = gmsh.model.occ.getSurfaceLoops(volume[1])
        for surfaceTag in surfaceTags[0]:
            gmsh.model.setColor([(2, surfaceTag)], 255, 12, 28)

    # 划分网格
    gmshFileFormatVersion = 2.2
    bgCellSizeField = None
    isDryRun = False

    mainConfig = configparser.ConfigParser()

    current_dir = os.path.abspath(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
    parent_dir = os.path.abspath(os.path.join(parent_dir, '..'))

    mainConfig.read(parent_dir + "\SRMConfig\GRAIN.ini", encoding='utf-8')

    GRAIN_INITBURN_PIECEs_Before = []
    GRAIN_SYM_PIECEs_Before = []
    GRAIN_HIL_PIECEs_Before = []
    n2DEnts = 0
    ens = gmsh.model.getEntities()
    for crtEntity in ens:
        crtDim = crtEntity[0]
        crtTag = crtEntity[1]
        crtColor = gmsh.model.getColor(crtDim, crtTag)
        if crtDim == 2:
            n2DEnts += 1
            physicalType = SRMMesh.PhysicalType.fromSurfaceColor(
                crtColor[0], crtColor[1], crtColor[2])
            crtSP = SRMMesh.SurfacePiece(crtDim, crtTag)

            if physicalType == SRMMesh.PhysicalType.GRAIN_INITBURN:
                GRAIN_INITBURN_PIECEs_Before.append(crtSP)
            elif physicalType == SRMMesh.PhysicalType.GRAIN_SYM:
                GRAIN_SYM_PIECEs_Before.append(crtSP)
            elif physicalType == SRMMesh.PhysicalType.GRAIN_HIL:
                GRAIN_HIL_PIECEs_Before.append(crtSP)
    logging.debug("%d initial burn faces in model before Boolean Fragment.",
                  len(GRAIN_INITBURN_PIECEs_Before))
    logging.debug("%d grain symmetry faces in model before Boolean Fragment.",
                  len(GRAIN_SYM_PIECEs_Before))
    logging.debug("%d grain-HIL faces in model before Boolean Fragment.",
                  len(GRAIN_HIL_PIECEs_Before))

    # 事先储存所有面的标号，因为synchronize之后面的标号会变化
    a = gmsh.model.occ.getEntities(2) + gmsh.model.occ.getEntities(2)
    # 存放模型所有面的颜色，与上述面的标号一一对应
    color = []
    for i in a:
        color.append(gmsh.model.getColor(2, i[1]))
    # 进行面之间的布尔合并运算， a存放的面的标号为【父面】，ovv存放的面的标号为其【子面】
    ov, ovv = gmsh.model.occ.fragment(
        gmsh.model.occ.getEntities(2), gmsh.model.occ.getEntities(2))
    gmsh.model.occ.synchronize()

    # 用于存放颜色字典，以供体合并后面颜色的查找
    SurfaceDict = []

    for index, i in enumerate(ovv):
        for j in i:
            # 把所有的【子面】创建面类
            sur = Surface(j[1])
            # 【子面】的坐标保存
            sur.save_point_message()
            # 【子面】的颜色保存。这里要检查子面是否同时在多个父面里面。如果在多个父面里面，子面颜色跟随【有颜色】的父面
            sur.color = color[index]
            for s in SurfaceDict:
                if s.surface_tag == j[1]:
                    sur.color = max(s.color, sur.color)
            SurfaceDict.append(sur)

    # 清空Gmsh，重新加载模型进行体合并
    gmsh.clear()

    gmsh.open(geometryPath)
    # 上色
    for volume in gmsh.model.occ.getEntities(3)[-1:]:
        surfaceLoopTags, surfaceTags = gmsh.model.occ.getSurfaceLoops(volume[1])
        for surfaceTag in surfaceTags[0]:
            gmsh.model.setColor([(2, surfaceTag)], 255, 128, 0)

    for volume in gmsh.model.occ.getEntities(3)[:-1]:
        surfaceLoopTags, surfaceTags = gmsh.model.occ.getSurfaceLoops(volume[1])
        for surfaceTag in surfaceTags[0]:
            gmsh.model.setColor([(2, surfaceTag)], 255, 12, 28)

    # 布尔运算是否允许一定误差
    if mainConfig["Tolerance"]["ToleranceFlag"] == "1":
        logging.debug("Start the Tolerance model.")
        gmsh.option.setNumber("Geometry.ToleranceBoolean", float(
            mainConfig["Tolerance"]["GeometryTolerance"]))
    if len(gmsh.model.occ.getEntities(3)) != 1:
        outDimTags, outDimTagsMap = gmsh.model.occ.fragment(gmsh.model.occ.getEntities(
            3), gmsh.model.occ.getEntities(3))
        gmsh.model.occ.synchronize()
        # 删除除药柱之外的部分
        gmsh.model.occ.remove(gmsh.model.occ.getEntities(3)[:-1], recursive=True)
        gmsh.model.occ.synchronize()
        gmsh.model.set_entity_name(3, gmsh.model.getEntities(3)[0][1], "grain")
        # gmsh.model.addPhysicalGroup(3, [gmsh.model.getEntities(3)[0][1]], -1,"grain")
        gmsh.model.occ.synchronize()
        gmsh.write("grain.step")

    # 用于存放体合并后所有面的信息
    SurfaceSet = []
    for surface in gmsh.model.occ.getEntities(2):
        temp = Surface(surface[1])
        temp.save_point_message()
        SurfaceSet.append(temp)

    # 面坐标是否允许一定误差范围
    tol = 0
    if mainConfig["Tolerance"]["ToleranceFlag"] == "1":
        tol = float(mainConfig["Tolerance"]["CoordinateTolerance"])

    # 体合并后，依据【字典】查询颜色
    for surface in SurfaceSet:
        for dict in SurfaceDict:
            # 两个面的坐标完全相同，该面的颜色即为【字典】中对应面的颜色
            if compare_surface(dict.coordinate, surface.coordinate, tol):
                surface.color = dict.color
                surface.set_color()
                gmsh.model.occ.synchronize()

    if showOCC:
        gmsh.fltk.run()

    # # 布尔运算后各类型面的数量
    # GRAIN_INITBURN_PIECEs = []
    # GRAIN_SYM_PIECEs = []
    # GRAIN_HIL_PIECEs = []
    # n2DEnts = 0
    # ens = gmsh.model.getEntities()
    # for crtEntity in ens:
    #     crtDim = crtEntity[0]
    #     crtTag = crtEntity[1]
    #     crtColor = gmsh.model.getColor(crtDim, crtTag)
    #
    #     if crtDim == 2:
    #         n2DEnts += 1
    #
    #         physicalType = SRMMesh.PhysicalType.fromSurfaceColor(
    #             crtColor[0], crtColor[1], crtColor[2])
    #         crtSP = SRMMesh.SurfacePiece(crtDim, crtTag)
    #
    #         if physicalType == SRMMesh.PhysicalType.GRAIN_INITBURN:
    #             GRAIN_INITBURN_PIECEs.append(crtSP)
    #         elif physicalType == SRMMesh.PhysicalType.GRAIN_SYM:
    #             GRAIN_SYM_PIECEs.append(crtSP)
    #         elif physicalType == SRMMesh.PhysicalType.GRAIN_HIL:
    #             GRAIN_HIL_PIECEs.append(crtSP)
    # logging.debug("%d initial burn faces in model after Boolean Fragment.",
    #               len(GRAIN_INITBURN_PIECEs))
    # logging.debug("%d grain symmetry faces in model after Boolean Fragment.",
    #               len(GRAIN_SYM_PIECEs))
    # logging.debug("%d grain-HIL faces in model after Boolean Fragment.",
    #               len(GRAIN_HIL_PIECEs))

    # # 判断布尔运算后面的数量是否大于等于布尔运算前的，如果不满足说明丢失面了，需要调用Tolerance
    toleranceIsNeeded = True
    # if len(GRAIN_INITBURN_PIECEs) >= len(GRAIN_INITBURN_PIECEs_Before) and \
    #         len(GRAIN_SYM_PIECEs) >= len(GRAIN_SYM_PIECEs_Before) and \
    #         len(GRAIN_HIL_PIECEs) >= len(GRAIN_HIL_PIECEs_Before):
    #     toleranceIsNeeded = False
    #
    # if toleranceIsNeeded:
    #     # 清空Gmsh，重新加载模型进行体合并
    #     gmsh.clear()
    #     gmsh.open(geometryPath)
    #     # 上色
    #     for volume in gmsh.model.occ.getEntities(3)[-1:]:
    #         surfaceLoopTags, surfaceTags = gmsh.model.occ.getSurfaceLoops(volume[1])
    #         for surfaceTag in surfaceTags[0]:
    #             gmsh.model.setColor([(2, surfaceTag)], 255, 128, 0)
    #
    #     for volume in gmsh.model.occ.getEntities(3)[:-1]:
    #         surfaceLoopTags, surfaceTags = gmsh.model.occ.getSurfaceLoops(volume[1])
    #         for surfaceTag in surfaceTags[0]:
    #             gmsh.model.setColor([(2, surfaceTag)], 255, 12, 28)
    #
    #     # 布尔运算是否允许一定误差
    #     logging.debug("Start the Tolerance model.")
    #     gmsh.option.setNumber("Geometry.ToleranceBoolean", float(
    #             mainConfig["Tolerance"]["GeometryTolerance"]))
    #     if len(gmsh.model.occ.getEntities(3)) != 1:
    #         ov, ovv = gmsh.model.occ.fragment(gmsh.model.occ.getEntities(
    #             3), gmsh.model.occ.getEntities(3))
    #         gmsh.model.occ.synchronize()
    #
    #     # 体合并后，依据【字典】查询颜色
    #     for surface in SurfaceSet:
    #         for dict in SurfaceDict:
    #             # 两个面的坐标完全相同，该面的颜色即为【字典】中对应面的颜色
    #             if compare_surface(dict.coordinate, surface.coordinate, tol):
    #                 surface.color = dict.color
    #                 surface.set_color()
    #                 gmsh.model.occ.synchronize()

    # read geometry file
    logging.info("Reading geometry from %s ...", geometryPath)

    logging.debug("Input model contains %d faces.",
                  len(gmsh.model.occ.getEntities(2)))

    logging.info(
        "Geometry.OCCScaling is set to %f to fit input CAD file", gmsh.option.getNumber("Geometry.OCCScaling"))

    # mesh size config
    BurnRefine_Flag = False
    # whatever default value, the defalt flag is False
    BurnRefine_NumPointsPerCurve = 100
    BurnRefine_SizeMin = 0.5
    BurnRefine_SizeMax = 4
    BurnRefine_DistMin = 8
    BurnRefine_DistMax = 30

    HILRefine_Flag = 1
    # whatever default value, the defalt flag is False
    HILRefine_NumPointsPerCurve = 100
    HILRefine_SizeMin = 0.5
    HILRefine_SizeMax = 4
    HILRefine_DistMin = 8
    HILRefine_DistMax = 30

    if bgCellSizeField is None:
        bgCellSizeField = float(mainConfig["Config"]["CellScale"])

        BurnRefine_Flag = (mainConfig["BurnRefine"]["BurnRefineFlag"] == "1")
        BurnRefine_NumPointsPerCurve = int(
            round(float(mainConfig["BurnRefine"]["NumPointsPerCurve"])))
        BurnRefine_SizeMin = float(mainConfig["BurnRefine"]["SizeMin"])
        BurnRefine_SizeMax = float(mainConfig["BurnRefine"]["SizeMax"])
        BurnRefine_DistMin = float(mainConfig["BurnRefine"]["DistMin"])
        BurnRefine_DistMax = float(mainConfig["BurnRefine"]["DistMax"])

        HILRefine_Flag = (mainConfig["HILRefine"]["HILRefineFlag"] == "1")
        HILRefine_NumPointsPerCurve = int(
            round(float(mainConfig["HILRefine"]["NumPointsPerCurve"])))
        HILRefine_SizeMin = float(mainConfig["HILRefine"]["SizeMin"])
        HILRefine_SizeMax = float(mainConfig["HILRefine"]["SizeMax"])
        HILRefine_DistMin = float(mainConfig["HILRefine"]["DistMin"])
        HILRefine_DistMax = float(mainConfig["HILRefine"]["DistMax"])

    else:
        # if there is bgCellSizeField command line, ignore all ini configs
        BurnRefine_Flag = False
        HILRefine_Flag = False
        pass

    logging.info("Background cell size: %f", bgCellSizeField)

    grainVolumePiecesToSearch = []
    hilVolumePiecesToSearch = []
    caseVolumePiecesToSearch = []

    # record the colume names generated from CAD
    grainPartsStr = mainConfig["GeoFlag"]["Grain"]
    if len(grainPartsStr) > 0:
        grainVolumePiecesToSearch = [SRMMesh.VolumePiece(
            n) for n in grainPartsStr.split("@")]
        for g in grainVolumePiecesToSearch:
            logging.info("Grain part: %s", g)

    hilPartsStr = mainConfig["GeoFlag"]["HIL"]
    if len(hilPartsStr) > 0:
        hilVolumePiecesToSearch = [SRMMesh.VolumePiece(
            n) for n in hilPartsStr.split("@")]
        for h in hilVolumePiecesToSearch:
            logging.info("Heat insulation part: %s", h)

    casePartsStr = mainConfig["GeoFlag"]["Case"]
    if len(casePartsStr) > 0:
        caseVolumePiecesToSearch = [SRMMesh.VolumePiece(
            n) for n in casePartsStr.split("@")]
        for c in caseVolumePiecesToSearch:
            logging.info("Case part: %s", c)

    # contains the SurfacePiece/VolumPiece objects which already have a valid dim/tag
    # 布尔运算后各类型面的数量
    GRAIN_INITBURN_PIECEs = []
    GRAIN_SYM_PIECEs = []
    GRAIN_HIL_PIECEs = []
    VOLUME_GRAIN_PIECEs = []
    VOLUME_HIL_PIECEs = []
    VOLUME_CASE_PIECEs = []

    # counter
    n2DEnts = 0
    n3DEnts = 0

    ens = gmsh.model.getEntities()
    for crtEntity in ens:
        crtDim = crtEntity[0]
        crtTag = crtEntity[1]
        crtColor = gmsh.model.getColor(crtDim, crtTag)

        if crtDim == 2:
            n2DEnts += 1
            # color debug code
            # logging.debug("Entity D-%d T-%d Corlor: [%f, %f, %f]",
            #              crtDim, crtTag, crtColor[0], crtColor[1], crtColor[2])
            physicalType = SRMMesh.PhysicalType.fromSurfaceColor(
                crtColor[0], crtColor[1], crtColor[2])
            crtSP = SRMMesh.SurfacePiece(crtDim, crtTag)

            if physicalType == SRMMesh.PhysicalType.GRAIN_INITBURN:
                GRAIN_INITBURN_PIECEs.append(crtSP)
            elif physicalType == SRMMesh.PhysicalType.GRAIN_SYM:
                GRAIN_SYM_PIECEs.append(crtSP)
            elif physicalType == SRMMesh.PhysicalType.GRAIN_HIL:
                GRAIN_HIL_PIECEs.append(crtSP)

        if crtDim == 3:
            n3DEnts += 1

            crtEntName = gmsh.model.getEntityName(crtDim, crtTag)
            logging.debug("Read 3D entity: %s", crtEntName)

            searchSuccess = False
            for idx in range(len(grainVolumePiecesToSearch)):
                if grainVolumePiecesToSearch[idx].testSetFlags(crtEntName, crtDim, crtTag):
                    # test hits, move current piece from grainVolumePiecesToSearch to VOLUME_GRAIN_PIECEs
                    VOLUME_GRAIN_PIECEs.append(
                        copy.deepcopy(grainVolumePiecesToSearch[idx]))
                    del grainVolumePiecesToSearch[idx]
                    searchSuccess = True
                    break

            for idx in range(len(hilVolumePiecesToSearch)):
                if hilVolumePiecesToSearch[idx].testSetFlags(crtEntName, crtDim, crtTag):
                    # test hits, move current piece from hilVolumePiecesToSearch to VOLUME_HIL_PIECEs
                    VOLUME_HIL_PIECEs.append(
                        copy.deepcopy(hilVolumePiecesToSearch[idx]))
                    del hilVolumePiecesToSearch[idx]
                    searchSuccess = True
                    break

            for idx in range(len(caseVolumePiecesToSearch)):
                if caseVolumePiecesToSearch[idx].testSetFlags(crtEntName, crtDim, crtTag):
                    # test hits, move current piece from caseVolumePiecesToSearch to VOLUME_CASE_PIECEs
                    VOLUME_CASE_PIECEs.append(
                        copy.deepcopy(caseVolumePiecesToSearch[idx]))
                    del caseVolumePiecesToSearch[idx]
                    searchSuccess = True
                    break

            if not searchSuccess:
                logging.warning(
                    "No geometry flag information is found for 3D entity %s", crtEntName)

    logging.debug("%d 2D entities and %d 3D entities", n2DEnts, n3DEnts)

    if isDryRun:
        gmsh.finalize()
        sys.exit(0)

    if len(GRAIN_INITBURN_PIECEs) > 0:
        GRAIN_INITBURN_GroupTag = gmsh.model.addPhysicalGroup(
            2, [p.tag for p in GRAIN_INITBURN_PIECEs])
        gmsh.model.setPhysicalName(2, GRAIN_INITBURN_GroupTag, str(
            SRMMesh.PhysicalType.GRAIN_INITBURN.value))
    logging.debug("%d initial burn faces in model.",
                  len(GRAIN_INITBURN_PIECEs))

    if len(GRAIN_SYM_PIECEs) > 0:
        GRAIN_SYM_GroupTag = gmsh.model.addPhysicalGroup(
            2, [p.tag for p in GRAIN_SYM_PIECEs])
        gmsh.model.setPhysicalName(2, GRAIN_SYM_GroupTag, str(
            SRMMesh.PhysicalType.GRAIN_SYM.value))
    logging.debug("%d grain symmetry faces in model", len(GRAIN_SYM_PIECEs))

    if len(GRAIN_HIL_PIECEs) > 0:
        GRAIN_HIL_GroupTag = gmsh.model.addPhysicalGroup(
            2, [p.tag for p in GRAIN_HIL_PIECEs])
        gmsh.model.setPhysicalName(2, GRAIN_HIL_GroupTag, str(
            SRMMesh.PhysicalType.GRAIN_HIL.value))
    logging.debug("%d grain-HIL faces in model", len(GRAIN_HIL_PIECEs))

    # = process grain volumes =
    if len(VOLUME_GRAIN_PIECEs) == 0:
        logging.error("Unable to find grain volume in the input model.")

    for p in VOLUME_GRAIN_PIECEs:
        logging.debug("Creat grain volume physical group for piece %s", str(p))
        crtPhysicalGroupTag = gmsh.model.addPhysicalGroup(3, [p.tag])
        gmsh.model.setPhysicalName(3, crtPhysicalGroupTag, str(
            SRMMesh.PhysicalType.VOLUME_GRAIN.value) + p.name)

    for p in VOLUME_HIL_PIECEs:
        logging.debug(
            "Creat heat insulation volume physical group for piece %s", str(p))
        crtPhysicalGroupTag = gmsh.model.addPhysicalGroup(3, [p.tag])
        gmsh.model.setPhysicalName(3, crtPhysicalGroupTag, str(
            SRMMesh.PhysicalType.VOLUME_HIL.value))
    if len(VOLUME_HIL_PIECEs) > 1:
        logging.warning("There is %d HIL pieces!", len(VOLUME_HIL_PIECEs))

    for p in VOLUME_CASE_PIECEs:
        logging.debug("Creat case volume physical group for piece %s", str(p))
        crtPhysicalGroupTag = gmsh.model.addPhysicalGroup(3, [p.tag])
        gmsh.model.setPhysicalName(3, crtPhysicalGroupTag, str(
            SRMMesh.PhysicalType.VOLUME_CASE.value))
    if len(VOLUME_CASE_PIECEs) > 1:
        logging.warning("There is %d Case pieces!", len(VOLUME_CASE_PIECEs))

    logging.debug("%d entities in model", len(ens))

    # == == == size field == == ==

    bgSizeFieldTag = gmsh.model.mesh.field.add("MathEval")
    gmsh.model.mesh.field.setString(bgSizeFieldTag, "F", str(bgCellSizeField))

    meshSizeFieldsList = [bgSizeFieldTag]

    if BurnRefine_Flag:
        burnSurfaceDistanceFieldTag = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumber(
            burnSurfaceDistanceFieldTag, "NumPointsPerCurve", BurnRefine_NumPointsPerCurve)
        gmsh.model.mesh.field.setNumbers(burnSurfaceDistanceFieldTag, "SurfacesList", [
            p.tag for p in GRAIN_INITBURN_PIECEs])
        burnThreSizeFieldTag = gmsh.model.mesh.field.add("Threshold")
        gmsh.model.mesh.field.setNumber(
            burnThreSizeFieldTag, "InField", burnSurfaceDistanceFieldTag)
        gmsh.model.mesh.field.setNumber(
            burnThreSizeFieldTag, "SizeMin", BurnRefine_SizeMin)
        gmsh.model.mesh.field.setNumber(
            burnThreSizeFieldTag, "SizeMax", BurnRefine_SizeMax)
        gmsh.model.mesh.field.setNumber(
            burnThreSizeFieldTag, "DistMin", BurnRefine_DistMin)
        gmsh.model.mesh.field.setNumber(
            burnThreSizeFieldTag, "DistMax", BurnRefine_DistMax)
        meshSizeFieldsList.append(burnThreSizeFieldTag)

    if HILRefine_Flag:
        HILSurfaceDistanceFieldTag = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumber(
            HILSurfaceDistanceFieldTag, "NumPointsPerCurve", HILRefine_NumPointsPerCurve)
        gmsh.model.mesh.field.setNumbers(HILSurfaceDistanceFieldTag, "SurfacesList", [
            p.tag for p in GRAIN_HIL_PIECEs])
        HILThreSizeFieldTag = gmsh.model.mesh.field.add("Threshold")
        gmsh.model.mesh.field.setNumber(
            HILThreSizeFieldTag, "InField", HILSurfaceDistanceFieldTag)
        gmsh.model.mesh.field.setNumber(
            HILThreSizeFieldTag, "SizeMin", HILRefine_SizeMin)
        gmsh.model.mesh.field.setNumber(
            HILThreSizeFieldTag, "SizeMax", HILRefine_SizeMax)
        gmsh.model.mesh.field.setNumber(
            HILThreSizeFieldTag, "DistMin", HILRefine_DistMin)
        gmsh.model.mesh.field.setNumber(
            HILThreSizeFieldTag, "DistMax", HILRefine_DistMax)
        meshSizeFieldsList.append(HILThreSizeFieldTag)

    finalSizeFieldTag = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(
        finalSizeFieldTag, "FieldsList", meshSizeFieldsList)
    gmsh.model.mesh.field.setAsBackgroundMesh(finalSizeFieldTag)

    # disable the independent mesh size factor on CAD elements
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)

    logging.info("Backgroud size filed number: #%d", bgSizeFieldTag)

    # == == == do mesh == == ==
    logging.info("Performing mesh operation, please be patient ...")

    gmsh.model.mesh.generate(3)
    logging.info("Tidy duplicate nodes in mesh ...")
    gmsh.model.mesh.removeDuplicateNodes()

    logging.info("Generating mesh finished")

    # == == == write data == == ==
    if gmshFileFormatVersion is not None:
        gmsh.option.setNumber("Mesh.MshFileVersion", gmshFileFormatVersion)

    logging.info("Writting mesh file %s", "out.msh")

    gmsh.write(meshSavePath)

    if showMesh:
        gmsh.fltk.run()

    gmsh.finalize()

# flag, anchor = anchor(500, 5, 50, 5, 0.7, 5)
# flag, yi = yi(42,77,4,4,211,4,325, 60, 62, 5)
# flag, multiHoles = multiHoles(500, 50, 10, 5)
# # flag, wheel = wheel(100, 24.5, 5, 49, 0.36, 2, 2, 2, 28, 500)
# flag, star = star(45, 2, 0.78, 5, 29, 500, 2)
# flag, slottedTube = slottedTube(100, 10, 500, 5, 5, 100)
# flag, wheelPort = wheelPort(15, 8, 100, 500, 5)
# display, start_display, add_menu, add_function_to_menu = init_display()
# display.DisplayShape(anchor, update=True)
# display.DisplayShape(yi, update=True)
# display.DisplayShape(star, update=True)
# display.DisplayShape(wheelPort, update=True)
# # display.DisplayShape(slottedTube, update=True)
# # display.DisplayShape(multiHoles, update=True)
# # display.DisplayShape(wheel, update=True)
# start_display()