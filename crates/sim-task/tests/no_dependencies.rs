//! The workspace uses no external crates.

#[test]
fn cargo_lock_has_no_external_crates() {
    let lock = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../Cargo.lock")).expect("Cargo.lock");
    // Crates from a registry or git carry a `source`; workspace members do not.
    let external: Vec<&str> = lock.lines().filter(|l| l.starts_with("source = ")).collect();
    assert!(external.is_empty(), "external crates in Cargo.lock: {external:?}");
}
