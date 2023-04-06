function f = hom_dynamics(x)
    f = [x(2); (x(1)-3) * (x(1)+1) * (x(1)+3)];
end