#! /bin/sh
camera_pose_type=$1

if [ -n "$camera_pose_type" ]
then
  echo "camera_pose_type is ${camera_pose_type}"
else
  echo "camera_pose_type is empty"
  exit
fi

usd_list="Collisiongroups Force KinematicBody MultipleScenes PaintBallEmitter ParticleCloth ParticleInflatableDemo TriangleMeshMultimaterial TriggerConveryor(OmniGraph)"
for usd in $usd_list
do
  echo "make dataset ${camera_pose_type} usd/demo/${usd}.usd"
  python "scripts/${camera_pose_type}_camera_set.py" $usd
  wait
done
