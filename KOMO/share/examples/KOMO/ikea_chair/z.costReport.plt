set key autotitle columnheader
set title 'costReport ( plotting sqrt(costs) )'
plot 'z.costReport' u 0:1 w l \
  ,'' u 0:2 w l \
  ,'' u 0:3 w l \
  ,'' u 0:4 w l \

