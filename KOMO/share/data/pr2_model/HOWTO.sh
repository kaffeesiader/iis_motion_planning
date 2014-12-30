cp $1 z.urdf
./urdf2ors.py > z1-raw.ors
#sed 's/mesh="pr2_model/mesh="/g' z.ors > pr2-1-org.ors
cat pr2_before.ors z1-raw.ors pr2_after.ors > z2-augmented.ors
ors_editor -file z2-augmented.ors -cleanOnly
mv z.ors z3-clean.ors

