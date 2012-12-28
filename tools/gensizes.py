#!/bin/env python2.4

import sys
import time
import subprocess
import traceback

def ratelimit(n=0,interval=0.0,timefn=time.time):
  def d(f):
    count=[n,interval,timefn]
    if interval>0.0: count[1]=count[2]()-interval
    def c(*args,**kwds):
      if n>0: count[0]=count[0]+1
      t=count[1]
      if interval>0.0: t=count[2]()
      if count[0]>=n and t-count[1]>=interval:
        count[0]=0
        count[1]=t
        f(*args,**kwds)
    return c
  return d

def subprocess_shell(c):
  p=subprocess.Popen(c,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,shell=True)
  retcode=p.wait()
  if retcode!=0:
    try:
      output=p.stdout.read()
    except:
      raise ValueError('Called process exited with return value %d' % (retcode))
    raise ValueError('Called process exited with return value %d, output follows:\n%s' % (retcode,output))
  return p.stdout.read()

def printfn(*args):
  print ' '.join(args)

if __name__=='__main__':
  rlprint=ratelimit(interval=15)(printfn)
  args=sys.argv[1:]

  if len(args)<1:
    print '''Usage: gensizes.py input-file output-file
  input-file: Bundler list file (.txt).
  output-file: Bundler list file with sizes (.txt). Corrupt images
               have a size of 0 0.
  --skip: Resumes an interrupted operation by skipping lines;
          it does not verify that the existing lines are valid!
'''
    sys.exit(1)

  skip=False
  while '--skip' in args:
    skip=True
    args.remove('--skip')

  ifile=args[0]
  ofile=args[1]
  l=0
  nl=int(subprocess_shell(r"""wc -l '%s'""" % ifile).split(' ',1)[0])
  if skip==True:
    nskip=int(subprocess_shell(r"""wc -l '%s'""" % ofile).split(' ',1)[0])
    print 'Skipping %d lines.' % nskip
  f=open(ifile,'rt')
  try:
    if skip==False: f2=open(ofile,'wt')
    else: f2=open(ofile,'at')
    try:
      work_units,work_done,work_t0=nl,0,time.time()
      while(True):
        line=f.readline().strip()
        if line=='': break
        if skip==False or l>=nskip:
          try:
            w,h=tuple(int(x) for x in subprocess_shell(r"""identify -format '%%w,%%h' '%s'""" % line).strip().split(','))
            f2.write('%s %d %d\n' % (line,w,h))
          except:
            traceback.print_exc()
            f2.write('%s %d %d\n' % (line,0,0))
        l=l+1
        work_done=work_done+1
        #rlprint('%d/%d' % (l,nl))
        rlprint('%d/%d, %g min remaining' % (work_done,work_units,(float(work_units)/work_done-1)*(time.time()-work_t0)/60.0))
    finally:
      f2.close()
  finally:
    f.close()
  print 'Success.'
