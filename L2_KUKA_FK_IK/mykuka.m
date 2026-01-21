% DH is 6x4
% i-th row of the matrix DH will contain the parameters [theta_i d_i a_i alpha_i] in this order.

% returns myrobot, the robot structure

function myrobot = mykuka(DH)
    %initialize each link w/ corresponding DH table values
    L1 = Link('revolute', 'd', DH(1,2), 'a', DH(1,3), 'alpha', DH(1,4));
    L2 = Link('revolute', 'd', DH(2,2), 'a', DH(2,3), 'alpha', DH(2,4));
    L3 = Link('revolute', 'd', DH(3,2), 'a', DH(3,3), 'alpha', DH(3,4));
    L4 = Link('revolute', 'd', DH(4,2), 'a', DH(4,3), 'alpha', DH(4,4));
    L5 = Link('revolute', 'd', DH(5,2), 'a', DH(5,3), 'alpha', DH(5,4));
    L6 = Link('revolute', 'd', DH(6,2), 'a', DH(6,3), 'alpha', DH(6,4));

    arr = [L1 L2 L3 L4 L5 L6];

    %form robot structure
    myrobot = SerialLink(arr, 'name', "KUKA");
end