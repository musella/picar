from optparse import OptionParser, make_option

import os
import sys
import subprocess
import shlex
import json
from pprint import pprint

# -------------------------------------------------------------------------------        
def shell_expand(string):
    if string:
        return os.path.expanduser( os.path.expandvars(string) )
    return None

# -------------------------------------------------------------------------------        
def shell_args(cmd):
    return [ shell_expand(a) for a in shlex.split(cmd) ]


# -------------------------------------------------------------------------------        
class MyCLApp(object):
    
    # -------------------------------------------------------------------------------        
    def __init__(self,option_list=[],default_cfg=None):

        self.cmdline_ = " ".join(sys.argv)
        self.procs_ = {}
        self.logs_ = {}
        
        parser = OptionParser(option_list=[make_option("-c","--config",action="store",type="string",
                                                       default=shell_expand(default_cfg)
                                                   )
                                       ]+option_list
                          )

        (self.options_, self.args_) = parser.parse_args()

    # -------------------------------------------------------------------------------        
    def run_shell(self,cmd,bkg=False,log=None):
        
        print cmd
        if log:
            if log in self.logs_:
                self.logs_[log].close()
            self.logs_[log] = open(log,"w+")
            log = self.logs_[log]
        if bkg:
            if cmd in self.procs_:
                self.procs_[cmd].kill()
            args = shell_args(cmd)

            sp = subprocess.Popen(args,stdout=log,stderr=log)
            self.procs_[cmd] = sp
        else:
            stdout=subprocess.PIPE
            stderr=subprocess.PIPE
            if log:
                stdout=log
                stderr=log
            sp = subprocess.Popen(shell_args(cmd),stdout=stdout,stderr=stderr)
            if not log:
                print(sp.stdout.read())
                print(sp.stderr.read())
            st = sp.returncode
            print("Retuned:", st)
            print()
        
        return sp
        
    # -------------------------------------------------------------------------------        
    def load_config(self):
        if self.options_.config:
            self.options_.config = shell_expand(self.options_.config)
            with open(self.options_.config) as fin:
                self.config_ = json.loads( fin.read() )
                return True
        self.config_ = None
        return False

    def save_config(self):
        if self.config_!= None  and self.options_.config != None:
            with open(self.options_.config,"w+") as fout:
                fout.write( json.dumps(self.config_)  )
                fout.write("\n")
                fout.close()
        
            

    # -------------------------------------------------------------------------------        
    def __call__(self,*args):
        self.run(*args)
    
    
# -------------------------------------------------------------------------------        
if __name__ == "__main__":
    
    class Test(MyCLApp):
        
        def __init__(self):
            super(Test,self).__init__()
            
        
        def run(self):
            self.load_config()
            pprint( self.options_ ) 
            pprint( self.args_ )
            
    test = Test()
    test()
    
