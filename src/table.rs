//! Pooltool 0.4-compatible six-pocket table construction.

use std::f64::consts::{FRAC_PI_4, PI};

use crate::math::Vec2;
use crate::model::{PocketId, TableSpec};

#[derive(Clone, Copy, Debug)]
pub(crate) enum CushionDirection {
    Side1,
    Side2,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct LinearCushion {
    pub id: &'static str,
    pub p1: Vec2,
    pub p2: Vec2,
    pub direction: CushionDirection,
    pub height: f64,
}

impl LinearCushion {
    pub(crate) fn normal(self) -> Vec2 {
        let delta = self.p2 - self.p1;
        Vec2::new(-delta.y, delta.x)
            .normalized()
            .expect("table cushion segments have nonzero length")
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct CircularCushion {
    pub id: &'static str,
    pub center: Vec2,
    pub radius: f64,
    pub height: f64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Pocket {
    pub id: PocketId,
    pub center: Vec2,
    pub radius: f64,
    pub depth: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct TableGeometry {
    pub linear: Vec<LinearCushion>,
    pub circular: Vec<CircularCushion>,
    pub pockets: [Pocket; 6],
}

impl TableGeometry {
    #[allow(
        clippy::many_single_char_names,
        clippy::manual_midpoint,
        clippy::too_many_lines
    )]
    pub(crate) fn new(spec: TableSpec) -> Self {
        let p = spec.pocket_table;
        let cw = p.cushion_width;
        let ca = (p.corner_pocket_angle + 45.0) * PI / 180.0;
        let sa = p.side_pocket_angle * PI / 180.0;
        let pw = p.corner_pocket_width;
        let sw = p.side_pocket_width;
        let rc = p.corner_jaw_radius;
        let rs = p.side_jaw_radius;
        let dc = rc / ((PI / 2.0 + ca) / 2.0).tan();
        let ds = rs / ((PI / 2.0 + sa) / 2.0).tan();
        let h = p.cushion_height;
        let w = spec.width;
        let l = spec.length;
        let q = FRAC_PI_4.cos();

        let linear = vec![
            line(
                "3",
                (0.0, pw * q + dc),
                (0.0, (l - sw) / 2.0 - ds),
                CushionDirection::Side2,
                h,
            ),
            line(
                "6",
                (0.0, (l + sw) / 2.0 + ds),
                (0.0, l - pw * q - dc),
                CushionDirection::Side2,
                h,
            ),
            line(
                "15",
                (w, pw * q + dc),
                (w, (l - sw) / 2.0 - ds),
                CushionDirection::Side1,
                h,
            ),
            line(
                "12",
                (w, (l + sw) / 2.0 + ds),
                (w, l - pw * q - dc),
                CushionDirection::Side1,
                h,
            ),
            line(
                "18",
                (pw * q + dc, 0.0),
                (w - pw * q - dc, 0.0),
                CushionDirection::Side2,
                h,
            ),
            line(
                "9",
                (pw * q + dc, l),
                (w - pw * q - dc, l),
                CushionDirection::Side1,
                h,
            ),
            line(
                "5",
                (-cw, (l + sw) / 2.0 - cw * sa.sin()),
                (-ds * sa.cos(), (l + sw) / 2.0 - ds * sa.sin()),
                CushionDirection::Side1,
                h,
            ),
            line(
                "4",
                (-cw, (l - sw) / 2.0 + cw * sa.sin()),
                (-ds * sa.cos(), (l - sw) / 2.0 + ds * sa.sin()),
                CushionDirection::Side2,
                h,
            ),
            line(
                "13",
                (w + cw, (l + sw) / 2.0 - cw * sa.sin()),
                (w + ds * sa.cos(), (l + sw) / 2.0 - ds * sa.sin()),
                CushionDirection::Side1,
                h,
            ),
            line(
                "14",
                (w + cw, (l - sw) / 2.0 + cw * sa.sin()),
                (w + ds * sa.cos(), (l - sw) / 2.0 + ds * sa.sin()),
                CushionDirection::Side2,
                h,
            ),
            line(
                "1",
                (pw * q - cw * ca.tan(), -cw),
                (pw * q - dc * ca.sin(), -dc * ca.cos()),
                CushionDirection::Side2,
                h,
            ),
            line(
                "2",
                (-cw, pw * q - cw * ca.tan()),
                (-dc * ca.cos(), pw * q - dc * ca.sin()),
                CushionDirection::Side1,
                h,
            ),
            line(
                "8",
                (pw * q - cw * ca.tan(), l + cw),
                (pw * q - dc * ca.sin(), l + dc * ca.cos()),
                CushionDirection::Side1,
                h,
            ),
            line(
                "7",
                (-cw, l - pw * q + cw * ca.tan()),
                (-dc * ca.cos(), l - pw * q + dc * ca.sin()),
                CushionDirection::Side2,
                h,
            ),
            line(
                "11",
                (w + cw, l - pw * q + cw * ca.tan()),
                (w + dc * ca.cos(), l - pw * q + dc * ca.sin()),
                CushionDirection::Side2,
                h,
            ),
            line(
                "10",
                (w - pw * q + cw * ca.tan(), l + cw),
                (w - pw * q + dc * ca.sin(), l + dc * ca.cos()),
                CushionDirection::Side1,
                h,
            ),
            line(
                "16",
                (w + cw, pw * q - cw * ca.tan()),
                (w + dc * ca.cos(), pw * q - dc * ca.sin()),
                CushionDirection::Side1,
                h,
            ),
            line(
                "17",
                (w - pw * q + cw * ca.tan(), -cw),
                (w - pw * q + dc * ca.sin(), -dc * ca.cos()),
                CushionDirection::Side2,
                h,
            ),
        ];

        let circular = vec![
            circle("1t", (pw * q + dc, -rc), rc, h),
            circle("2t", (-rc, pw * q + dc), rc, h),
            circle("4t", (-rs, l / 2.0 - sw / 2.0 - ds), rs, h),
            circle("5t", (-rs, l / 2.0 + sw / 2.0 + ds), rs, h),
            circle("7t", (-rc, l - pw * q - dc), rc, h),
            circle("8t", (pw * q + dc, l + rc), rc, h),
            circle("10t", (w - pw * q - dc, l + rc), rc, h),
            circle("11t", (w + rc, l - pw * q - dc), rc, h),
            circle("13t", (w + rs, l / 2.0 + sw / 2.0 + ds), rs, h),
            circle("14t", (w + rs, l / 2.0 - sw / 2.0 - ds), rs, h),
            circle("16t", (w + rc, pw * q + dc), rc, h),
            circle("17t", (w - pw * q - dc, -rc), rc, h),
        ];

        let corner_offset = p.corner_pocket_depth / 2.0_f64.sqrt();
        let side_offset = p.side_pocket_depth;
        let pockets = [
            pocket(
                PocketId::LeftBottom,
                (-corner_offset, -corner_offset),
                p.corner_pocket_radius,
                p.pocket_depth,
            ),
            pocket(
                PocketId::LeftCenter,
                (-side_offset, l / 2.0),
                p.side_pocket_radius,
                p.pocket_depth,
            ),
            pocket(
                PocketId::LeftTop,
                (-corner_offset, l + corner_offset),
                p.corner_pocket_radius,
                p.pocket_depth,
            ),
            pocket(
                PocketId::RightBottom,
                (w + corner_offset, -corner_offset),
                p.corner_pocket_radius,
                p.pocket_depth,
            ),
            pocket(
                PocketId::RightCenter,
                (w + side_offset, l / 2.0),
                p.side_pocket_radius,
                p.pocket_depth,
            ),
            pocket(
                PocketId::RightTop,
                (w + corner_offset, l + corner_offset),
                p.corner_pocket_radius,
                p.pocket_depth,
            ),
        ];
        Self {
            linear,
            circular,
            pockets,
        }
    }
}

fn line(
    id: &'static str,
    p1: (f64, f64),
    p2: (f64, f64),
    direction: CushionDirection,
    height: f64,
) -> LinearCushion {
    LinearCushion {
        id,
        p1: Vec2::new(p1.0, p1.1),
        p2: Vec2::new(p2.0, p2.1),
        direction,
        height,
    }
}

fn circle(id: &'static str, center: (f64, f64), radius: f64, height: f64) -> CircularCushion {
    CircularCushion {
        id,
        center: Vec2::new(center.0, center.1),
        radius,
        height,
    }
}

fn pocket(id: PocketId, center: (f64, f64), radius: f64, depth: f64) -> Pocket {
    Pocket {
        id,
        center: Vec2::new(center.0, center.1),
        radius,
        depth,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_table_has_pooltool_component_counts() {
        let geometry = TableGeometry::new(TableSpec::default());
        assert_eq!(geometry.linear.len(), 18);
        assert_eq!(geometry.circular.len(), 12);
        assert_eq!(geometry.pockets.len(), 6);
    }
}
