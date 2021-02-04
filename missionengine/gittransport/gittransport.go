package gittransport

import (
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"time"

	sssh "golang.org/x/crypto/ssh"

	//"github.com/go-git/go-billy/v5/memfs"
	git "github.com/go-git/go-git/v5"
	"github.com/go-git/go-git/v5/plumbing/transport/ssh"
	"gopkg.in/yaml.v2"
	//"github.com/go-git/go-git/v5/storage/memory"
)

// type Config struct {
// 	Cloud struct {
// 		Git string `yaml:"git"`
// 	} `yaml:"cloud"`
// 	Fleet []struct {
// 		Name string `yaml:"name"`
// 		Git  string `yaml:"git"`
// 	}
// }

type ConfigDrone struct {
	Name             string
	GitServerAddress string
	GitServerKey     string
	GitClientKey     string
}
type Config struct {
	Wifi struct {
		SSID   string
		Secret string
	}
	Drones []ConfigDrone `yaml:",omitempty"`
}

type GitEngine struct {
	Config           Config
	gitServerAddress string
	gitServerKey     string
	// gitSigner        sssh.Signer
	// r             *git.Repository
	flagName      string
	fileChanges   map[string]time.Time
	filePositions map[string]int64
}

func (cfg *Config) GetFleetNames() []string {
	result := make([]string, 0)
	for _, x := range cfg.Drones {
		result = append(result, x.Name)
	}

	return result
}

func (me GitEngine) DataDir() string {
	return "db/" + me.flagName
}

func New(gitServerAddress string, gitServerKey string) *GitEngine {
	flagName := time.Now().Format("20060102150405")
	// signer, _ := sssh.NewSignerFromSigner(gitSigner)

	// repository := cloneRepository(gitServerAddress, flagName)
	cloneRepository(gitServerAddress, flagName)

	config := parseConfig(flagName)

	return &GitEngine{
		config,
		gitServerAddress,
		gitServerKey,
		// signer,
		flagName,
		make(map[string]time.Time),
		make(map[string]int64),
	}
}

/*
func (m *GitEngine) CommitAll() {
	w, err := m.r.Worktree()
	if err != nil {
		log.Fatal(err)
	}

	err = w.AddWithOptions(&git.AddOptions{All: true})
	if err != nil {
		log.Fatalf("add all: %v", err)
	}

	_, err = w.Commit("task "+m.flagName, &git.CommitOptions{
		Author: &object.Signature{
			Name:  m.flagName,
			Email: m.flagName + "@auto-fleet",
			When:  time.Now(),
		},
	})
	if err != nil {
		log.Fatal(err)
	}
}

func (m *GitEngine) Commit(f string) {
	w, err := m.r.Worktree()
	if err != nil {
		log.Fatal(err)
	}

	filename := strings.TrimPrefix(f, m.DataDir())
	// filename := "db/" + m.flagName + "/" + name
	log.Printf("GIT ADD: file %v", filename)
	// file, err := os.Create(filename)
	// file.Close()
	// if err != nil {
	// 	log.Fatalf("create file: %v", err)
	// }
	_, err = w.Add(filename)
	if err != nil {
		log.Fatalf("add file: %v", err)
	}

	_, err = w.Commit("task "+m.flagName, &git.CommitOptions{
		Author: &object.Signature{
			Name:  m.flagName,
			Email: m.flagName + "@auto-fleet",
			When:  time.Now(),
		},
	})
	if err != nil {
		log.Fatal(err)
	}
}
*/

func (m *GitEngine) CommitAll() {
}

func (m *GitEngine) pullFiles() bool {
	gitSSHCommand := "ssh -i /fog-drone/ssh/id_rsa -o \"IdentitiesOnly=yes\" -o \"UserKnownHostsFile=/fog-drone/ssh/known_host_cloud\""
	cloneCmd := exec.Command("git", "pull", "--rebase")
	cloneCmd.Env = []string{"GIT_SSH_COMMAND=" + gitSSHCommand}
	cloneCmd.Dir = "db/" + m.flagName
	cloneOut, err := cloneCmd.CombinedOutput()
	if err != nil {
		log.Printf("%s\n\nCould not clone: %v", cloneOut, err)
		return false
	}

	return true
}

func cloneRepository(gitServerAddress string, flagName string) {
	gitSSHCommand := "ssh -i /fog-drone/ssh/id_rsa -o \"IdentitiesOnly=yes\" -o \"UserKnownHostsFile=/fog-drone/ssh/known_host_cloud\""
	repoAddr := fmt.Sprintf("ssh://git@%s/fleet.git", gitServerAddress)
	cloneCmd := exec.Command("git", "clone", repoAddr, "db/"+flagName)
	cloneCmd.Env = []string{"GIT_SSH_COMMAND=" + gitSSHCommand}
	cloneOut, err := cloneCmd.CombinedOutput()
	if err != nil {
		log.Printf("%s\n\nCould not clone: %v", cloneOut, err)
		return
	}
	// log.Printf("%s", cloneOut)
	// o, e := exec.Command("ls", "-la", "fleet-db").CombinedOutput()
	// log.Printf("%s", o)
	// if e != nil {
	// 	log.Printf("err: %v", e)
	// }
}

func cloneRepository_old(gitServerAddress string, gitSigner sssh.Signer, flagName string) *git.Repository {
	// sshKey, err := ioutil.ReadFile("id_ed25519")
	// if err != nil {
	// 	log.Fatalf("sshkey file: %v", err)
	// }

	publicKey := &ssh.PublicKeys{User: "git", Signer: gitSigner}

	r, err := git.PlainClone("db/"+flagName, false, &git.CloneOptions{
		Auth:     publicKey,
		URL:      gitServerAddress,
		Progress: os.Stdout,
	})
	if err != nil {
		log.Fatalf("clone: %v", err)
	}

	return r
	// _ = r

	// var myGitAddress string
	// var otherGitAddress string
	// var otherDroneName string
	// for _, d := range configFile.Fleet {
	// 	if d.Name == *flagName {
	// 		myGitAddress = d.Git
	// 	} else {
	// 		otherGitAddress = d.Git
	// 		otherDroneName = d.Name
	// 	}
	// }
	// if myGitAddress == "" {
	// 	log.Fatal("Could not find my git address")
	// }
	// s := strings.Split(myGitAddress, ":")
	// log.Printf("my git address: %v:%v", s[0], s[1])
	// log.Printf("other git address: %v", otherGitAddress)

	// add another drone remote
	// _, err = r.CreateRemote(&config.RemoteConfig{
	// 	Name: otherDroneName,
	// 	URLs: []string{"ssh://git@" + otherGitAddress + "/mission"},
	// })
	// if err != nil {
	// 	log.Fatal(err)
	// }

	// go RunSSHServer(s[1])
	// go RunPullPushCycles(flagName, r)
}

func parseConfig(flagName string) Config {
	configBytes, err := ioutil.ReadFile("db/" + flagName + "/config.yaml")
	if err != nil {
		log.Fatalf("read config: %v", err)
	}
	var configFile Config
	err = yaml.Unmarshal(configBytes, &configFile)
	if err != nil {
		log.Fatalf("unmarshal config: %v", err)
	}

	return configFile
}

/*
func (m *GitEngine) pullFiles() bool {
	r := m.r
	// sshKey, err := ioutil.ReadFile("id_ed25519")
	// if err != nil {
	// 	log.Fatalf("sshkey file: %v", err)
	// }

	// publicKey, err := ssh.NewPublicKeys("git", sshKey, "")
	// if err != nil {
	// 	log.Fatalf("sshkey: %v", err)
	// }

	publicKey := &ssh.PublicKeys{User: "git", Signer: m.gitSigner}

	w, err := r.Worktree()
	if err != nil {
		log.Fatal(err)
	}

	headBefore, err := r.Reference("refs/remotes/origin/master", true)
	if err != nil {
		log.Fatal(err)
	}

	err = pullWithMerge(m.flagName, w, r, &git.PullOptions{
		Auth:       publicKey,
		RemoteName: "origin",
		Progress:   os.Stdout,
	})
	if err != nil && err != git.NoErrAlreadyUpToDate {
		log.Printf("Could not pull: %v", err)
	}

	headAfter, err := r.Reference("refs/remotes/origin/master", true)
	if err != nil {
		log.Fatal(err)
	}

	if headBefore.Hash().String() != headAfter.Hash().String() {
		return true
	}

	return false
}

func (m *GitEngine) pushFiles() bool {
	r := m.r
	sshKey, err := ioutil.ReadFile("id_ed25519")
	if err != nil {
		log.Fatalf("sshkey file: %v", err)
	}

	publicKey, err := ssh.NewPublicKeys("git", sshKey, "")
	if err != nil {
		log.Fatalf("sshkey: %v", err)
	}

	// w, err := r.Worktree()
	// if err != nil {
	// 	log.Fatal(err)
	// }

	err = r.Push(&git.PushOptions{
		Auth:       publicKey,
		RemoteName: "origin",
		Progress:   os.Stdout,
	})
	if err != nil {
		log.Printf("Could not push: %v", err)
	}

	return true
}

func updateHEAD(r *git.Repository, commit plumbing.Hash) error {
	head, err := r.Storer.Reference(plumbing.HEAD)
	if err != nil {
		return err
	}

	name := plumbing.HEAD
	if head.Type() != plumbing.HashReference {
		name = head.Target()
	}

	ref := plumbing.NewHashReference(name, commit)
	return r.Storer.SetReference(ref)
}

func isFastForward(s storer.EncodedObjectStorer, old, new plumbing.Hash) (bool, error) {
	c, err := object.GetCommit(s, new)
	if err != nil {
		return false, err
	}

	found := false
	iter := object.NewCommitPreorderIter(c, nil, nil)
	err = iter.ForEach(func(c *object.Commit) error {
		if c.Hash != old {
			return nil
		}

		found = true
		return storer.ErrStop
	})
	return found, err
}

func identifyMergeFiles(commitHead *object.Commit, commitAncestor *object.Commit) (map[string]*object.Tree, error) {
	filemap := make(map[string]*object.Tree)
	iter := object.NewCommitPreorderIter(commitHead, nil, nil)
	err := iter.ForEach(func(c *object.Commit) error {
		//log.Printf("Processing %v", c.Hash)
		if c.Hash == commitAncestor.Hash {
			//log.Printf("Found common ancestor, stopping..")
			return storer.ErrStop
		}
		t, err := c.Tree()
		if err != nil {
			return err
		}
		//log.Printf("Tree: %+v", t)

		fileIter := t.Files()
		defer fileIter.Close()
		for {
			f, err := fileIter.Next()
			if err == io.EOF {
				break
			}
			if err != nil {
				return err
			}
			//log.Printf("file: %v", f.Name)
			if filemap[f.Name] == nil {
				filemap[f.Name] = t
			}
		}
		return nil
	})

	return filemap, err
}

func pullWithMerge(flagName string, w *git.Worktree, r *git.Repository, o *git.PullOptions) error {
	if err := o.Validate(); err != nil {
		return err
	}
	log.Printf("Pull with merge")

	remote, err := r.Remote(o.RemoteName)
	if err != nil {
		return err
	}

	err = remote.Fetch(&git.FetchOptions{
		RemoteName: o.RemoteName,
		Depth:      o.Depth,
		Auth:       o.Auth,
		Progress:   o.Progress,
		Force:      o.Force,
	})

	updated := true
	if err == git.NoErrAlreadyUpToDate {
		updated = false
	} else if err != nil {
		return err
	}

	// ref = other branch
	//ref, err := storer.ResolveReference(remote, o.ReferenceName)
	ref, err := r.Reference("refs/remotes/origin/master", true)
	if err != nil {
		return err
	}

	// head = current branch
	head, err := r.Head()
	if err == nil {
		// commits...ref(remote)...head(local) (or ref == head)
		headAheadOfRef, err := isFastForward(r.Storer, ref.Hash(), head.Hash())
		if err != nil {
			return err
		}
		if !updated && headAheadOfRef {
			return git.NoErrAlreadyUpToDate
		}

		// commits..head(local)..ref(remote)
		ff, err := isFastForward(r.Storer, head.Hash(), ref.Hash())
		if err != nil {
			return err
		}
		if !ff {
			// non-fast-forward, remote is ahead of local

			// files
			// find common ancestor
			commitHead, err := r.CommitObject(head.Hash())
			if err != nil {
				return err
			}
			commitRef, err := r.CommitObject(ref.Hash())
			if err != nil {
				return err
			}
			xx, err := commitHead.MergeBase(commitRef)
			if err != nil {
				return err
			}
			if len(xx) == 0 {
				return errors.New("merge: no common ancestor!")
			}
			if len(xx) > 1 {
				return errors.New("merge: too many ancestors!")
			}

			//log.Printf("Identify HEAD merge files")
			filemapHead, err := identifyMergeFiles(commitHead, xx[0])
			if err != nil {
				w.Reset(&git.ResetOptions{
					Mode:   git.MergeReset,
					Commit: head.Hash(),
				})
				return err
			}
			//log.Printf("Identify REF merge files")
			filemapRef, err := identifyMergeFiles(commitRef, xx[0])
			if err != nil {
				w.Reset(&git.ResetOptions{
					Mode:   git.MergeReset,
					Commit: head.Hash(),
				})
				return err
			}
			//log.Printf("head files: %+v", filemapHead)
			//log.Printf("ref files: %+v", filemapRef)

			// no need to check which files are conflicted
			// as no files should ever be..
			//for f, t := range filemapHead {
			//	if filemapRef[f] != nil {
			//		return errors.New("Conflict")
			//	}
			//	_ = t
			//}
			// TODO: if ever any file changes during operation we should check
			// if we should use remote or local here.
			for f, t := range filemapRef {
				if filemapHead[f] != nil {
					// the file already exists - do nothing
				}

				// new file
				//copyTo, err := filemapHead[f].File(f)
				//if err != nil {
				//return err
				//}
				copyFrom, err := t.File(f)
				if err != nil {
					return err
				}
				src, err := copyFrom.Reader()
				if err != nil {
					return err
				}
				defer src.Close()
				dest, err := w.Filesystem.Create(f)
				if err != nil {
					return err
				}
				defer dest.Close()
				_, err = io.Copy(dest, src)
				if err != nil {
					return err
				}
			}

			// commit
			mergeCommitHash, err := w.Commit("Merge with Drone123", &git.CommitOptions{
				Author: &object.Signature{
					Name:  flagName,
					Email: flagName + "@auto-fleet",
					When:  time.Now(),
				},
				Parents: []plumbing.Hash{ref.Hash(), head.Hash()},
			})
			if err != nil {
				return err
			}
			if err := w.Reset(&git.ResetOptions{
				Mode:   git.MergeReset,
				Commit: mergeCommitHash,
			}); err != nil {
				return err
			}
			return nil
		}
	}
	if err != nil && err != plumbing.ErrReferenceNotFound {
		return err
	}
	if err := updateHEAD(r, ref.Hash()); err != nil {
		return err
	}

	if err := w.Reset(&git.ResetOptions{
		Mode:   git.MergeReset,
		Commit: ref.Hash(),
	}); err != nil {
		return err
	}
	//

	// 	err := w.Pull(&git.PullOptions{
	// 		RemoteName: "origin",
	// 		Progress:   os.Stdout,
	// 	})
	// 	if err != nil && err != git.NoErrAlreadyUpToDate {
	// 		log.Fatal(err)
	// 	}
	//
	return nil
}
*/
