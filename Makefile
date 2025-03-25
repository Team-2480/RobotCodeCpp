# We aren't actually using make, I just am putting maitenance macros here instead of writing a script.

.PHONY: build gp CompDbRegen linkCompDb run clean

# Replace gradle with make lol
build:
	./gradlew build

# Gah! gradlew is not executable again
# Extreme shorthand for gradle permissions
gp:
	chmod +x ./gradlew

# Regenerate compile_commands
regenDB:
	./gradlew generateCompileCommands

# this link is for linux only, here we can link to compile_commands instead of manually copying it.
linkDB:
	- rm compile_commands.json
	ln -s ./build/TargetedCompileCommands/linuxx86-64debug/compile_commands.json compile_commands.json

# Replace gradle with make lol
run:
	./gradlew deploy

clean:
	# Clean garbage here:
