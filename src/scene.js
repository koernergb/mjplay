// Self-contained MJCF: primitive geoms only, no external mesh assets.
// A free-floating capsule "arm" plus loose blocks are dropped from a height
// onto a floor and a fixed obstacle box. Free joints let everything actually
// fall and collide, which guarantees ncon > 0 within the first second.
// M1+ replaces this with MuJoCo Menagerie models (which pull in mesh files).
export const SCENE_XML = `
<mujoco model="m0probe">
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <default>
    <geom friction="1 0.05 0.05" solref="0.01 1"/>
  </default>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.30 0.33 0.40 1"/>
    <geom name="obstacle" type="box" pos="0.2 0 0.15" size="0.15 0.15 0.15"
          rgba="0.45 0.42 0.50 1"/>

    <body name="arm" pos="0 0 1.0" euler="0 20 0">
      <freejoint name="free_arm"/>
      <geom name="g_arm" type="capsule" fromto="-0.25 0 0 0.25 0 0" size="0.05"
            rgba="0.55 0.68 1.0 1"/>
    </body>

    <body name="ball" pos="0.1 0.05 1.4">
      <freejoint name="free_ball"/>
      <geom name="g_ball" type="sphere" size="0.09" rgba="0.65 0.78 1.0 1"/>
    </body>

    <body name="cube" pos="-0.15 -0.05 0.7">
      <freejoint name="free_cube"/>
      <geom name="g_cube" type="box" size="0.08 0.08 0.08" rgba="0.75 0.85 1.0 1"/>
    </body>
  </worldbody>
</mujoco>`;
