''' Animation helper class '''


''' 
Each dataunit can have its own figure or draw on the same figure as the rest
However, each class instance will have its own set of data    

Inspired by: https://stackoverflow.com/questions/40126176/fast-live-plotting-in-matplotlib-pyplot
'''
import matplotlib.pyplot as plt
import time

class Animate:
    # TODO: set default values
    def __init__(self,number=1, xlim=(10,10), ylim=(10,10), gridSize=1, linewidth=5, markerType='o', markerSize=5, sleep=0.005, order=10):
        '''
            Grabs the current figure instance based on number
        '''
        self.xlim = xlim
        self.ylim = ylim
        # Only add subplot if initializing!
        # TODO: Add legend during init phase
        if not plt.fignum_exists(number):
            # Figure doesnt exist, so initialize stuff
            fig = plt.figure(number)
            ax1 = fig.add_subplot(1,1,1)
            if len(xlim) != 0 and len(ylim) != 0:
                ax1.set_xlim(self.xlim[0], self.xlim[1])
                ax1.set_ylim(self.ylim[0], self.ylim[1])
            else:
                ax1.set_autoscalex_on(True)
                ax1.set_autoscaley_on(True)
        else:
            fig = plt.figure(number)
            #ax1 = fig.gca()
            ax1 = fig.add_subplot(1,1,1)

        # markerType = 'xc'
        self.lines, = ax1.plot([],[],markerType, markersize=markerSize, linewidth=linewidth)
        ax1.set_zorder(order)
        ax1.patch.set_visible(False)    #may not be necessary

        #ax1.grid(True)

        # note that the first draw comes before setting data
        fig.canvas.draw() 

        #self.axbackground = fig.canvas.copy_from_bbox(ax1.bbox)
        plt.show(block=False)

        # data holder
        self.xdata = []
        self.ydata = []

        # sleep amount  (secs)
        self.sleep = sleep  

        #Listen to mouse click for pausing
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.pause = False

        # set things for class-wide access
        self.fig = fig
        self.ax1 = ax1   

    '''update the figure '''    
    def update(self,new_data):
        ''' new_data is a tuple of (x,y)  consist of updated data
        '''
        self.xdata.append(new_data[0])
        self.ydata.append(new_data[1])
        self.lines.set_xdata(self.xdata)
        self.lines.set_ydata(self.ydata)
        
        #sleep
        time.sleep((self.sleep))

        # cache background
        #self.axbackground = self.fig.canvas.copy_from_bbox(self.ax1.bbox)

        #self.fig.canvas.restore_region(self.axbackground)
        if self.pause == True:
            plt.waitforbuttonpress()

        # Bug when saving animation, keep ax size fixed
        self.ax1.set_xlim(self.xlim[0], self.xlim[1])
        self.ax1.set_ylim(self.ylim[0], self.ylim[1])

        self.ax1.draw_artist(self.lines)
        #self.fig.canvas.blit(self.ax1.bbox)
        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    def onclick(self, event):
        self.pause ^= True
        #fig.canvas.mpl_disconnect(cid)


from matplotlib.animation import FFMpegWriter
''' Helper class to help save animations '''
class SaveAnimation:
    def __init__(self, figNumber, outfile):      
        #self.fig = plt.gcf()    #assuming we are focused on 1 plt
        self.fig = plt.figure(figNumber)
        metadata = dict(title='Movie Test', artist='Matplotlib', comment='Movie support!')
        self.writer = FFMpegWriter(fps=15, metadata=metadata)
        self.writer.setup(self.fig, outfile, dpi=100)

    def update(self):
        #self.fig = animateObject.fig #may not be necessary here...
        self.writer.grab_frame()

    def save(self):
        self.writer.finish()