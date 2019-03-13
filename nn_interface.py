# -*- coding: utf-8 -*-
import os
os.environ['CHAINER_TYPE_CHECK'] = '0'  # chainerの処理時に型チェックを行わない。若干の高速化が期待できる

import chainer
import chainer.functions as F
import numpy
import cupy
import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.font_manager
import matplotlib.lines
import pickle

m2mdir = '/home/d-kiki/motion2motion/sliding/'
nndir = m2mdir + 'outputs/IROS18/'
import sys
sys.path.append(m2mdir)
import model, trajectory

chainer.using_config('train', False)
xp = numpy
start_of_motion = False
end_of_motion = True

nn = None
model_name = 'pizza3a_epoch_50'
#model_name = 'pizza0a_epoch_50'
#model_name = 'pizza5a_epoch_50'
n_units = 1024
chunksize = 100
motion_length = 1201

Xsin = []
Usin = []
Xdin = []
Udin = []

Xs = []
Us = []
us_chunk = []
xs_chunk = []
xs_offset = xp.zeros((1, 4), dtype=xp.float32)

Xd = []
Ud = []
ud_chunk = []
xd_chunk = []
xd_offset = xp.zeros((1, 4), dtype=xp.float32)


def init_nn():
    global nn

    # Setup an NN
    nn = model.Motion2motion(n_units=n_units, chunksize=chunksize)

    # If specified, load existing data
    chainer.serializers.load_npz(nndir + model_name, nn)

    # Get input trajectories
    global Xsin, Usin, Xdin, Udin
    Xsin, Usin, Xdin, Udin = trajectory.sliding2_det(motion_length, 1, level=1.0, seed=16)
    #Xsin, Usin, Xdin, Udin = trajectory.sliding2_det(motion_length, 1, level=1.0, seed=17)
    #Xsin, Usin, Xdin, Udin = trajectory.sliding2_det(motion_length, 1, level=1.0, seed=21)
    Xsin = xp.array(Xsin)
    Usin = xp.array(Usin)
    Xdin = xp.array(Xdin)
    Udin = xp.array(Udin)

    # Encode
    nn.encode(Xdin)

    # Initialize variables
    global Xs, Us, xs_chunk
    Xs = []
    Us = []
    xs_chunk = []
    global Xd, Ud, xd_chunk
    Xd = []
    Ud = []
    xd_chunk = []
    global start_of_motion, end_of_motion
    start_of_motion = True
    end_of_motion = False

    print("NN-INTERFACE: Initialized.")
    return 0


def input_state(x):
    global nn
    global Xs, Xd
    global xs_chunk, xd_chunk
    global xs_offset, xd_offset
    global us_chunk
    global start_of_motion, end_of_motion

    if start_of_motion:
        xs_offset = xp.array([[x[0], x[1], 0.0, 0.0]], dtype=xp.float32)
        xd_offset = xp.array([[x[4], x[5], 0.0, 0.0]], dtype=xp.float32)
        xs_chunk = [xp.zeros((1, 4), dtype=xp.float32)]*chunksize
        xd_chunk = [xp.zeros((1, 4), dtype=xp.float32)]*chunksize
        xs = xp.array([x[0:4]], dtype=xp.float32) - xs_offset
        xd = xp.array([x[4:8]], dtype=xp.float32) - xd_offset
        Xs.append(xs)
        Xd.append(xd)
        start_of_motion = False
    else:
        xs = xp.array([x[0:4]], dtype=xp.float32) - xs_offset
        xd = xp.array([x[4:8]], dtype=xp.float32) - xd_offset
        xs_chunk.append(xs)
        xd_chunk.append(xd)
        Xs.append(xs)
        Xd.append(xd)

    if len(xd_chunk) >= chunksize:
        us_means, us_vars = nn.decode(xp.concatenate(xd_chunk + xs_chunk, axis=1))
        us_chunk = xp.split(us_means.data, chunksize, axis=1)
        xs_chunk = []
        xd_chunk = []

    if len(Xs) >= Xsin.shape[1]:
        end_of_motion = True  # デコーダ出力が終了したことを示すフラグを立てる
        print("NN-INTERFACE: Motion output ends.")

        Xsout = numpy.array(Xs).reshape(-1, 4)
        Xdout = numpy.array(Xd).reshape(-1, 4)
        dat = {'Xin': Xdin, 'Xs': Xsout, 'Xd': Xdout}
        with open('output_{}.pickle'.format(model_name), mode='wb') as f:
            pickle.dump(dat, f)
        draw_graph()

    return 0


def receive_action():
    global us_chunk

    if len(us_chunk) > 0:
        ret = us_chunk.pop(0)
        return ret.flatten().tolist()
    else:
        print('NN-INTERFACE: Output chunk is ran out.')
        return [0.0, 0.0]


def is_end_of_motion():
    if end_of_motion:
        return 1
    else:
        return 0


def quit_generate():
    global Xs, Xd
    global end_of_motion

    if end_of_motion:
        return

    end_of_motion = True  # デコーダ出力が終了したことを示すフラグを立てる
    print("NN-INTERFACE: Motion output quits.")

    Xsout = numpy.array(Xs).reshape(-1, 4)
    Xdout = numpy.array(Xd).reshape(-1, 4)
    dat = {'Xin': Xdin, 'Xs': Xsout, 'Xd': Xdout}
    with open('output_{}.pickle'.format(model_name), mode='wb') as f:
        pickle.dump(dat, f)
    draw_graph()


def draw_graph():
    Xsout = numpy.array(Xs).reshape(-1, 4)
    Xdout = numpy.array(Xd).reshape(-1, 4)

    matplotlib.pyplot.rcParams['font.size'] = 16

    fig1 = matplotlib.pyplot.figure()
    ax1 = fig1.add_subplot(1,1,1)
    ax1.plot(Xsout.T[0],  Xsout.T[1],  '-',  lw=2, c='r', label='response')
    #ax1.plot(Xsin[0,:,0], Xsin[0,:,1], '--', lw=1, c='b', label='command')
    fig1.canvas.set_window_title("Spatula position")
    ax1.set_xlabel("$x$ [m]", fontsize=20)
    ax1.set_ylabel("$y$ [m]", fontsize=20)
    ax1.legend()
    fig1.subplots_adjust(left=0.19, right=0.97, top=0.95, bottom=0.15)

    fig2 = matplotlib.pyplot.figure()
    ax2 = fig2.add_subplot(1,1,1)
    ax2.plot(Xdout.T[0],  Xdout.T[1],  '-',  lw=2, c='r', label='response')
    ax2.plot(Xdin[0,:,0], Xdin[0,:,1], '--', lw=1, c='b', label='command')
    fig2.canvas.set_window_title("Object position")
    ax2.set_xlabel("$x$ [m]", fontsize=20)
    ax2.set_ylabel("$y$ [m]", fontsize=20)
    ax2.legend()
    fig2.subplots_adjust(left=0.19, right=0.97, top=0.95, bottom=0.15)

    matplotlib.pyplot.show()


def sliding2(T=2000, parallel=1, mu=0.1, mu_max=0.5, level=0.0):
    from scipy import interpolate

    Us = []
    Ud = []
    mug = 1.0*9.8

    for i in range(parallel):
        if numpy.random.rand() >= level:
            # Generate acceleration of the spatula
            us = numpy.zeros((T, 2))
            t = 0
            while t < T:
                ur = mug + numpy.random.rand()*5
                uth = numpy.random.rand() * 6.283
                ux = ur * numpy.cos(uth)
                uy = ur * numpy.sin(uth)
                cycle = numpy.random.randint(10, 50)
                us[t:cycle, 0] = ux
                us[t:cycle, 1] = uy
                us[t+cycle:t+4*cycle, 0] = -ux
                us[t+cycle:t+4*cycle, 1] = -uy
                t += 4*cycle

            # Generate acceleration of the object
            vdi = numpy.zeros((2,), dtype=numpy.float32)
            ud = []
            for usi in us:
                udi = physics.contact_model_xp(usi, vdi, mu=mu, mu_max=mu_max)
                vdi = vdi + udi*dT
                ud.append(udi)

            Us.append(us)
            Ud.append(ud)
        else:
            c = (numpy.random.rand(2, 7) - 0.5) * 2.0
            c[0][0] = 0.0
            c[1][0] = 0.0

            fx, _ = interpolate.splprep(c, k=2)
            ud = interpolate.splev(trajectory.smooth_time(T), fx)
            ud = numpy.array(ud, dtype=numpy.float32)
            us = ud * 0.0

            Us.append(us.T)
            Ud.append(ud.T)

    Us = numpy.array(Us, dtype=numpy.float32)
    Ud = numpy.array(Ud, dtype=numpy.float32)

    # Generate state vectors
    Xs = numpy.zeros((parallel, T+1, 4), dtype=numpy.float32)
    Xd = numpy.zeros((parallel, T+1, 4), dtype=numpy.float32)
    for t in range(T):
        Xs[:, t+1, :] = Xs[:, t, :].dot(trajectory.A.T) + Us[:, t, :].dot(trajectory.B.T)
        Xd[:, t+1, :] = Xd[:, t, :].dot(trajectory.A.T) + Ud[:, t, :].dot(trajectory.B.T)

    return Xs, Us, Xd, Ud


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('model_name',               type=str,
                                                    help='Name of NN.')
    parser.add_argument('--length', '-l',           type=int, default=500)
    parser.add_argument('--n-units', '-u',          type=int, default=64,
                                                    help='Number of units in each hidden layer.')
    parser.add_argument('--chunksize',              type=int, default=50,
                                                    help='Number of samples in a chunk.')
    parser.add_argument('--gpu', '-g',              type=int, default=-1,
                                                    help='Use GPU for training.')
    parser.add_argument('--reversed',               action='store_true',
                                                    help='Reverse input sequences during encoding.')
    args = parser.parse_args()

    chainer.using_config('train', False)
    print('#name =', args.model_name)
    print('#n_units =', args.n_units)
    outdir = './outputs'
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    # Setup an NN
    nn = model.Motion2motion(n_units=args.n_units, chunksize=args.chunksize)
    #nn.reversed = args.reversed

    # If specified, load existing data
    chainer.serializers.load_npz(args.model_name, nn)

    # Select CPU or GPU
    global xp
    if args.gpu >= 0:
        xp = cupy
        chainer.cuda.get_device(args.gpu).use()
        nn.to_gpu()
    else:
        xp = numpy

    matplotlib.pyplot.rcParams['font.size'] = 24
    colors = ['orange', 'r', 'b', 'cyan', 'm', 'k', 'crimson', 'navy', 'lime',
              'darkgreen', 'saddlebrown', 'indigo', 'olive', 'tan', 'palevioletred', 'springgreen', 'steelblue', 'gold',
              'darkgreen', 'saddlebrown', 'indigo', 'olive', 'tan', 'palevioletred', 'springgreen', 'steelblue', 'gold']

    fig1 = matplotlib.pyplot.figure()
    ax1 = fig1.add_subplot(1,1,1)

    fig2 = matplotlib.pyplot.figure()
    ax2 = fig2.add_subplot(1,1,1)

    fig3 = matplotlib.pyplot.figure()
    ax3 = fig3.add_subplot(1,1,1)

    N = 5

    pbar1 = tqdm.tqdm(total=N, unit='trajectories')
    for i in range(N):
        # Get input trajectories
        Xsin, Usin, Xdin, Udin = trajectory.sliding2(args.length, 1, 1.0)
        #Xsin, Usin, Xdin, Udin = trajectory.sliding1(args.length, 1)
        Xsin = xp.array(Xsin)
        Usin = xp.array(Usin)
        Xdin = xp.array(Xdin)
        Udin = xp.array(Udin)

        # Encode
        nn.encode(Xdin)

        # Decode
        # Initialize variables for the spatula
        Xs = []
        Us = []
        xs_chunk = [xp.zeros((1, 4), dtype=xp.float32)]*args.chunksize
        xs_mean = chainer.Variable(xp.zeros((1, 4), dtype=xp.float32))
        xs_var  = chainer.Variable(xp.zeros((1, 4, 4), dtype=xp.float32))
        Xs.append((xs_mean, xs_var))

        # Initialize variables for the object
        Xd = []
        Ud = []
        xd_chunk = [xp.zeros((1, 4), dtype=xp.float32)]*args.chunksize
        xd_mean = chainer.Variable(xp.zeros((1, 4), dtype=xp.float32))
        xd_var  = chainer.Variable(xp.zeros((1, 4, 4), dtype=xp.float32))
        Xd.append((xd_mean, xd_var))

        pbar_time = tqdm.tqdm(total=args.length-1, unit='samples')
        for t in range(args.length-1):
            # Generate an output chunk
            if t % args.chunksize == 0:
                us_means, us_vars = nn.decode(xp.concatenate(xd_chunk + xs_chunk, axis=1))
                xs_chunk = []
                xd_chunk = []

            # Get acceleration of the spatula
            #us_mean = us_means[t%args.chunksize]
            #us_var = us_vars[t%args.chunksize]
            us_mean = us_means[:, t%args.chunksize, :]
            us_var = us_vars[:, t%args.chunksize, :, :]

            # Get acceleration of the object by the contact model
            ud_mean = physics.contact_model(us_mean, xd_mean[:, 2:], mu=0.1, mu_max=0.5)
            ud_var = us_var

            # Get state vectors by the state equation
            xs_mean, xs_var = physics.state_equation(xs_mean, xs_var, us_mean, us_var)
            xd_mean, xd_var = physics.state_equation(xd_mean, xd_var, ud_mean, ud_var)

            # Record outputs
            Us.append((us_mean, us_var))
            Xs.append((xs_mean, xs_var))
            Ud.append((ud_mean, ud_var))
            Xd.append((xd_mean, xd_var))

            # Add a sample to the next input chunk
            xs_chunk.append(xs_mean.data)
            xd_chunk.append(xd_mean.data)

            pbar_time.update(1)
        pbar_time.close()

        Usout = []
        for us in Us[:-1]:
            Usout.append(chainer.cuda.to_cpu(us[0].data.flatten()))
        Usout = numpy.asarray(Usout)

        Udout = []
        for ud in Ud[:-1]:
            Udout.append(chainer.cuda.to_cpu(ud[0].data.flatten()))
        Udout = numpy.asarray(Udout)

        Xsout = []
        for xs in Xs:
            Xsout.append(chainer.cuda.to_cpu(xs[0].data.flatten()))
        Xsout = numpy.asarray(Xsout)

        Xdout = []
        for xd in Xd:
            Xdout.append(chainer.cuda.to_cpu(xd[0].data.flatten()))
        Xdout = numpy.asarray(Xdout)

        ax1.plot(Xsout.T[0],  Xsout.T[1],  '-',  lw=2, c=colors[i], label='generated')
        ax1.plot(Xsin[0,:,0], Xsin[0,:,1], '--', lw=1, c=colors[i], label='true')

        ax2.plot(Xdout.T[0],  Xdout.T[1],  '-',  lw=2, c=colors[i], label='generated')
        ax2.plot(Xdin[0,:,0], Xdin[0,:,1], '--', lw=1, c=colors[i], label='true')

        ax3.plot(Usout.T[0],  Usout.T[1],  '-',  lw=2, c=colors[i], label='generated')
        ax3.plot(Usin[0,:,0], Usin[0,:,1], '--', lw=1, c=colors[i], label='true')

        pbar1.update(1)
    pbar1.close()

    fig1.canvas.set_window_title("Spatula position")
    ax1.set_xlabel("$x$ [m]", fontsize=20)
    ax1.set_ylabel("$y$ [m]", fontsize=20)
    #original_line = ax1.plot([], [], color='k', ls='--', lw=1, label='original trajectories')
    #deformed_line = ax1.plot([], [], color='k', ls='-',  lw=2, label='deformed trajectories')
    #ax1.legend([original_line, deformed_line], bbox_to_anchor=(1.03, 1), loc='upper left', borderaxespad=0)

    fig2.canvas.set_window_title("Object position")
    ax2.set_xlabel("$x$ [m]", fontsize=20)
    ax2.set_ylabel("$y$ [m]", fontsize=20)

    fig3.canvas.set_window_title("Spatula acceleration")
    ax3.set_xlabel("$a_x$ [m/s/s]", fontsize=20)
    ax3.set_ylabel("$a_y$ [m/s/s]", fontsize=20)

    matplotlib.pyplot.show()


if __name__ == '__main__':
    pass
