"""
rho-zipper / steering-only-at-tau=0 demonstration in Manim.

Shows a "head" (circle + nose) at the present moment tau=0, with ghost
copies at past and future tau-offsets along a horizontal tau-axis.

Each ghost head at offset Delta-tau displays the steering function
angle_at(t + Delta-tau). When the steering function jolts at the
central tau=0, every ghost shows the same jolt at its own time-shift
-- visually demonstrating that the trails are rigid pose-extruded
structures that get dragged by the tau=0 steering.

The jolt visibly propagates through the tau-axis from future-ghosts
(right, violet) to past-ghosts (left, blue) as scene time advances.
That wave-through-the-trail is *the* visual signature of "steering
happens only at tau=0; the rest is rigid drag."

Usage:
    pip install manim
    manim -pql rhozipper.py RhoZipper    # quick low-res preview (480p)
    manim -pqh rhozipper.py RhoZipper    # high-res (1080p)
    manim -pqk rhozipper.py RhoZipper    # 4k

Tweak knobs:
    angle_at(t)    -- steering function (yaw vs absolute time)
    TAU_VALUES     -- which tau-offsets to show as ghosts
    X_PER_TAU      -- how spread out the trail is on screen
    The two run_time values in the JOLT stage control how slowly the
    wave propagates through the trail. Larger = more legible.
"""

from manim import *


class RhoZipper(Scene):
    def construct(self):
        # =====================================================
        # 1. STEERING FUNCTION
        #    Head's yaw at absolute time t.
        #    Three phases:
        #      [0, 3)    -- smooth ramp 0 deg -> 30 deg
        #      [3, 3.3)  -- JOLT to 40 deg (the rigid-drag event)
        #      [3.3, +)  -- held at 40 deg
        # =====================================================
        def angle_at(t):
            if t < 0:
                return 0
            if t < 3:
                return 30 * DEGREES * (t / 3)
            if t < 3.3:
                return 30 * DEGREES + 10 * DEGREES * ((t - 3) / 0.3)
            return 40 * DEGREES

        # =====================================================
        # 2. tau-AXIS LAYOUT
        # =====================================================
        TAU_VALUES = [-0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6]  # seconds
        X_PER_TAU = 7.0   # 1 second of tau -> 7 manim units horizontally
        Y_BASE = 0.3

        def tau_pos(tau):
            return RIGHT * (tau * X_PER_TAU) + UP * Y_BASE

        def tau_opacity(tau):
            if abs(tau) < 1e-3:
                return 1.0
            return max(0.18, 0.7 - abs(tau) * 0.85)

        def tau_color(tau):
            if tau < -0.01:
                return BLUE     # past tau -- blue
            if tau > 0.01:
                return PURPLE   # future tau -- violet
            return YELLOW       # present tau=0 -- yellow

        # =====================================================
        # 3. HEAD MESH
        #    Just a filled circle (head) + a dot offset to
        #    +x (the "nose"), rotated and translated.
        # =====================================================
        def make_head(angle_rad, position, opacity, color):
            body = Circle(
                radius=0.32,
                color=color,
                fill_color=color,
                fill_opacity=opacity * 0.5,
                stroke_opacity=opacity,
                stroke_width=2.5,
            )
            nose = Dot(
                point=RIGHT * 0.45,
                color=color,
                radius=0.09,
                fill_opacity=opacity,
            )
            head = VGroup(body, nose)
            head.rotate(angle_rad, about_point=ORIGIN)
            head.shift(position)
            return head

        # =====================================================
        # 4. STATIC SCENE: tau-AXIS, TICKS, LABELS, TITLE
        # =====================================================
        axis = Line(
            tau_pos(-0.75), tau_pos(0.75),
            color=GREY_B, stroke_width=2,
        )

        ticks = VGroup(*[
            Line(
                tau_pos(tau) + DOWN * 0.08,
                tau_pos(tau) + UP * 0.08,
                color=GREY_B, stroke_width=2,
            )
            for tau in TAU_VALUES if abs(tau) > 1e-3
        ])

        zero_tick = Line(
            tau_pos(0) + DOWN * 0.20,
            tau_pos(0) + UP * 0.20,
            color=YELLOW, stroke_width=5,
        )

        past_lbl = Text("past τ", font_size=24, color=BLUE).next_to(
            axis, LEFT, buff=0.25
        )
        future_lbl = Text("future τ", font_size=24, color=PURPLE).next_to(
            axis, RIGHT, buff=0.25
        )
        zero_lbl = VGroup(
            Text("τ = 0", font_size=28, color=YELLOW),
            Text(
                "(steering happens here)",
                font_size=18, color=YELLOW, slant=ITALIC,
            ),
        ).arrange(DOWN, buff=0.06).next_to(zero_tick, UP, buff=0.20)

        title = Text(
            "ρ-zipper: steering only at τ=0",
            font_size=34,
        ).to_edge(UP, buff=0.4)

        # =====================================================
        # 5. TIME TRACKER + ALWAYS_REDRAW HEADS
        # =====================================================
        time = ValueTracker(0.0)

        head_group = VGroup(*[
            always_redraw(
                lambda tau=tau: make_head(
                    angle_rad=angle_at(time.get_value() + tau),
                    position=tau_pos(tau),
                    opacity=tau_opacity(tau),
                    color=tau_color(tau),
                )
            )
            for tau in TAU_VALUES
        ])

        # =====================================================
        # 6. PLAYBACK
        # =====================================================
        # Stage 1: title + tau-axis
        self.play(Write(title))
        self.play(
            Create(axis), Create(ticks), Create(zero_tick),
            Write(past_lbl), Write(future_lbl), Write(zero_lbl),
            run_time=2,
        )
        self.wait(0.4)

        # Stage 2: heads appear
        # (using self.add not FadeIn because always_redraw mobjects
        #  don't play nicely with opacity-based animations)
        self.add(head_group)
        self.wait(0.6)

        # Stage 3: smooth steering 0 -> 30 deg over 3 seconds
        narration1 = Text(
            "Smooth steering at τ=0: angle ramps 0° → 30° over 3 s.",
            font_size=24, color=WHITE,
        ).to_edge(DOWN, buff=0.5)
        self.play(FadeIn(narration1))
        self.play(time.animate.set_value(3.0), run_time=3, rate_func=linear)
        self.wait(0.3)

        # Stage 4: the JOLT.
        # The jolt itself is only 0.3 abstract-seconds; we play it over
        # 2.5 real seconds so the wave-through-trail is legible.
        # The future ghosts (right) jolt FIRST because they show the
        # head's pose at (t + tau) for tau > 0 -- i.e., what's about to
        # happen at tau=0. Past ghosts jolt LAST. The rigid-drag wave
        # propagates right-to-left through the trail.
        narration2 = Text(
            "JOLT at τ=0 — every ghost is a time-shift of\n"
            "the central head, so the entire trail visibly drags.",
            font_size=24, color=YELLOW, line_spacing=0.8,
        ).to_edge(DOWN, buff=0.5)
        self.play(ReplacementTransform(narration1, narration2))
        self.play(time.animate.set_value(3.3), run_time=2.5, rate_func=linear)
        self.wait(0.5)

        # Stage 5: post-jolt -- the jolt continues to propagate into
        # past ghosts as scene time advances past 3.3.
        narration3 = Text(
            "Held at 40°. The past trail (blue) is still\n"
            "registering the jolt at its τ-offset.",
            font_size=24, color=WHITE, line_spacing=0.8,
        ).to_edge(DOWN, buff=0.5)
        self.play(ReplacementTransform(narration2, narration3))
        self.play(time.animate.set_value(3.9), run_time=2.0, rate_func=linear)
        self.wait(0.8)

        # Stage 6: takeaway
        takeaway = Text(
            "Steering force lives only at τ=0.\n"
            "The trails are rigid pose-extrusion that gets dragged.",
            font_size=28, color=WHITE, line_spacing=0.85,
        ).to_edge(DOWN, buff=0.5)
        self.play(ReplacementTransform(narration3, takeaway))
        self.wait(3)