
l = 0.08
r = 0.04
m = 0.5

Ixx = m * r**2 / 4.0 + m * l**2 / 12.0
Izz = m * r**2 / 2.0

print('l={}, r={}, m={}'.format(l, r, m))
print('Ixx = {}'.format(Ixx))
print('Izz = {}'.format(Izz))
print()


a = 0.5
b = 0.5
c = 0.15
m = 0.01

Ixx = m * ((b/2)**2 + (c/2)**2) / 3
Iyy = m * ((c/2)**2 + (a/2)**2) / 3
Izz = m * ((a/2)**2 + (b/2)**2) / 3

print('a={}, b={}, c={}'.format(a, b, c))
print('Ixx = {}'.format(Ixx))
print('Iyy = {}'.format(Iyy))
print('Izz = {}'.format(Izz))
